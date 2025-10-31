import pyrealsense2 as rs
import numpy as np
import cv2
from collections import deque
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
import threading
import time
from scipy import stats
import serial

class TemporalFilter:
    def __init__(self, history_size=10, confidence_threshold=0.7):
        self.tip_history = deque(maxlen=history_size)
        self.confidence_threshold = confidence_threshold
        
    def calculate_confidence(self, current_tip, points_3d):
        """Calculate confidence based on tip consistency and point density"""
        if len(points_3d) < 10:
            return 0.0
            
        # Confidence based on number of points (more points = more confident)
        point_confidence = min(1.0, len(points_3d) / 50.0)
        
        # Confidence based on tip position consistency
        position_confidence = 1.0
        if len(self.tip_history) > 3:
            recent_tips = list(self.tip_history)[-3:]
            distances = [np.linalg.norm(current_tip - tip) for tip in recent_tips]
            avg_distance = np.mean(distances)
            # Lower confidence if tip jumps around too much (>5mm average jump)
            position_confidence = max(0.0, 1.0 - (avg_distance / 0.005))
        
        return (point_confidence + position_confidence) / 2.0
    
    def temporal_filter(self, current_tip, confidence):
        """Apply temporal filtering to smooth detections"""
        # Add current detection to history
        self.tip_history.append(current_tip)
        
        # Only use high-confidence detections for filtering
        high_conf_tips = []
        
        # Get recent high-confidence detections
        for i in range(len(self.tip_history)):
            if i < len(self.tip_history) - 5:  # Only look at recent 5 frames
                continue
            # For simplicity, assume recent detections are more reliable
            high_conf_tips.append(self.tip_history[i])
        
        if len(high_conf_tips) == 0:
            return current_tip
        
        # If confidence is too low, use more smoothing
        if confidence < self.confidence_threshold:
            # Heavy smoothing: use more historical data
            weights = np.exp(np.linspace(-2, 0, len(high_conf_tips)))
        else:
            # Light smoothing: trust current detection more
            weights = np.exp(np.linspace(-0.5, 0, len(high_conf_tips)))
        
        weights = weights / np.sum(weights)
        
        # Weighted average
        filtered_tip = np.average(high_conf_tips, weights=weights, axis=0)
        
        return filtered_tip

class WireTipDetectorNode(Node):
    def __init__(self):
        super().__init__('wire_tip_detector')
        
        # New services for SEQ1 and SEQ2
        self.srv_seq1 = self.create_service(
            Trigger, 
            'execute_seq1', 
            self.execute_seq1_callback
        )
        
        self.srv_seq2 = self.create_service(
            Trigger, 
            'execute_seq2', 
            self.execute_seq2_callback
        )
        self.srv_relay = self.create_service(
            Trigger, 
            'toggle_relay', 
            self.toggle_relay_callback
            )
        
        # Data collection variables
        self.collecting_data = False
        self.collected_x_coords = []
        self.collection_start_time = None
        self.collection_duration = 3.0  # Fixed 3 second collection time
        self.data_lock = threading.Lock()
        self.collection_complete_event = threading.Event()  # Event for synchronization
        
        # Detection variables
        self.current_tip_data = None
        self.detection_active = True
        
        # Initialize Arduino serial communication
        try:
            self.arduino = serial.Serial('/dev/ttyACM1', 9600, timeout=1)  # Adjust port as needed
            time.sleep(2)  # Wait for Arduino to initialize
            self.get_logger().info('Arduino connection established')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Arduino: {str(e)}')
            self.arduino = None
        
        # Initialize RealSense and detection components
        self.setup_realsense()
        self.setup_detection()
        
        self.get_logger().info('Wire tip detector node initialized - CV2 windows will show')
        self.get_logger().info('Services available:')
        self.get_logger().info('  ros2 service call /execute_seq1 std_srvs/srv/Trigger')
        self.get_logger().info('  ros2 service call /execute_seq2 std_srvs/srv/Trigger')
        
        # Start detection thread
        self.detection_thread = threading.Thread(target=self.run_detection, daemon=True)
        self.detection_thread.start()
    
    
    
    def toggle_relay_callback(self, request, response):
        """Toggle relay state"""
        if self.arduino is None:
            response.success = False
            response.message = "Arduino not connected"
            return response
        
        self.get_logger().info('\033[92mToggling relay state\033[0m')  # Green text
        
        try:
            self.arduino.write(b"TOGGLE_RELAY\n")
            
            # Read Arduino response
            arduino_response = ""
            timeout_start = time.time()
            while time.time() - timeout_start < 2.0:  # 2 second timeout
                if self.arduino.in_waiting:
                    arduino_response += self.arduino.read(self.arduino.in_waiting).decode('utf-8', errors='ignore')
                time.sleep(0.1)
            
            response.success = True
            response.message = f"Relay toggled. Arduino response: {arduino_response.strip()}"
            
        except Exception as e:
            self.get_logger().error(f'Arduino communication error: {str(e)}')
            response.success = False
            response.message = f"Arduino communication failed: {str(e)}"
        
        return response
    
    def setup_realsense(self):
        """Initialize RealSense camera"""
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        self.depth_scale = self.pipeline.get_active_profile().get_device().first_depth_sensor().get_depth_scale()
        
        # Configurable depth range (in meters)
        self.MIN_DEPTH_M = 0.19  # 19 cm
        self.MAX_DEPTH_M = 0.21  # 21 cm
    
    def setup_detection(self):
        """Initialize detection filters"""
        self.temporal_filter_log = TemporalFilter(history_size=10, confidence_threshold=0.6)
        self.temporal_filter_sobel = TemporalFilter(history_size=10, confidence_threshold=0.6)
    
    def remove_outliers_x(self, x_coords, z_threshold=2.0):
        """Remove outliers from X coordinates using z-score method"""
        if len(x_coords) < 3:
            return x_coords
        
        x_coords = np.array(x_coords)
        z_scores = np.abs(stats.zscore(x_coords))
        
        # Remove points where z-score > threshold
        mask = z_scores < z_threshold
        filtered_coords = x_coords[mask]
        
        return filtered_coords.tolist()
    
    def execute_seq1_callback(self, request, response):
        """Execute Sequence 1: Immediately send SET_X 0 and SEQ1 to Arduino. Data collection logic is commented out."""
        if self.arduino is None:
            response.success = False
            response.message = "Arduino not connected"
            return response

        self.get_logger().info('\033[91mExecuting SEQ1 - Sending SET_X 0 and SEQ1 to Arduino\033[0m')  # Red text
        try:
            # Immediately send SET_X 0
            set_x_command = "SET_X 0\n"
            self.arduino.write(set_x_command.encode())
            time.sleep(0.1)  # Short delay

            # Send SEQ1 command
            self.get_logger().info('\033[91mSending SEQ1 command to Arduino\033[0m')  # Red text
            self.arduino.write(b"SEQ1\n")

            # Read Arduino response
            arduino_response = ""
            timeout_start = time.time()
            while time.time() - timeout_start < 5.0:  # 5 second timeout
                if self.arduino.in_waiting:
                    arduino_response += self.arduino.read(self.arduino.in_waiting).decode('utf-8', errors='ignore')
                time.sleep(0.1)

            response.success = True
            response.message = f"SEQ1 executed with X=0. Arduino response: {arduino_response.strip()}"

        except Exception as e:
            self.get_logger().error(f'Arduino communication error: {str(e)}')
            response.success = False
            response.message = f"Arduino communication failed: {str(e)}"

        # --- Data collection and X calculation logic below is commented out ---
        # self.get_logger().info('\033[91mExecuting SEQ1 - Starting X coordinate collection\033[0m')  # Red text
        # with self.data_lock:
        #     self.collecting_data = True
        #     self.collected_x_coords = []
        #     self.collection_start_time = time.time()
        #     self.collection_complete_event.clear()
        # timeout_occurred = not self.collection_complete_event.wait(timeout=self.collection_duration + 2.0)
        # if timeout_occurred:
        #     self.get_logger().error('Data collection timed out')
        #     with self.data_lock:
        #         self.collecting_data = False
        #     response.success = False
        #     response.message = "Data collection timed out"
        #     return response
        # with self.data_lock:
        #     if len(self.collected_x_coords) == 0:
        #         response.success = False
        #         response.message = "No X coordinates collected"
        #         return response
        #     filtered_x_coords = self.remove_outliers_x(self.collected_x_coords)
        #     if len(filtered_x_coords) == 0:
        #         response.success = False
        #         response.message = f"All {len(self.collected_x_coords)} collected coordinates were outliers"
        #         return response
        #     averaged_x_mm = np.mean(filtered_x_coords) * 1000
        #     self.get_logger().info(f'\033[91mSending SET_X {averaged_x_mm:.2f} to Arduino\033[0m')  # Red text
        #     try:
        #         set_x_command = f"SET_X {averaged_x_mm:.2f}\n"
        #         self.arduino.write(set_x_command.encode())
        #         time.sleep(0.1)  # Short delay
        #         self.get_logger().info('\033[91mSending SEQ1 command to Arduino\033[0m')  # Red text
        #         self.arduino.write(b"SEQ1\n")
        #         arduino_response = ""
        #         timeout_start = time.time()
        #         while time.time() - timeout_start < 5.0:  # 5 second timeout
        #             if self.arduino.in_waiting:
        #                 arduino_response += self.arduino.read(self.arduino.in_waiting).decode('utf-8', errors='ignore')
        #             time.sleep(0.1)
        #         response.success = True
        #         removed_outliers = len(self.collected_x_coords) - len(filtered_x_coords)
        #         response.message = f"SEQ1 executed with X={averaged_x_mm:.2f}mm from {len(filtered_x_coords)} samples ({removed_outliers} outliers removed). Arduino response: {arduino_response.strip()}"
        #     except Exception as e:
        #         self.get_logger().error(f'Arduino communication error: {str(e)}')
        #         response.success = False
        #         response.message = f"Arduino communication failed: {str(e)}"
        return response

    def execute_seq2_callback(self, request, response):
        """Execute Sequence 2"""
        if self.arduino is None:
            response.success = False
            response.message = "Arduino not connected"
            return response
        
        self.get_logger().info('\033[91mExecuting SEQ2 - Opening both grippers\033[0m')  # Red text
        
        try:
            self.arduino.write(b"SEQ2\n")
            
            # Read Arduino response
            arduino_response = ""
            timeout_start = time.time()
            while time.time() - timeout_start < 3.0:  # 3 second timeout
                if self.arduino.in_waiting:
                    arduino_response += self.arduino.read(self.arduino.in_waiting).decode('utf-8', errors='ignore')
                time.sleep(0.1)
            
            response.success = True
            response.message = f"SEQ2 executed. Arduino response: {arduino_response.strip()}"
            
        except Exception as e:
            self.get_logger().error(f'Arduino communication error: {str(e)}')
            response.success = False
            response.message = f"Arduino communication failed: {str(e)}"
        
        return response
    
    def process_with_detector(self, gray, depth_image, depth_frame, intr, temporal_filter, detector_name, detector_edges, color_image):
        """Process detection with a specific edge detector and return overlay and detection data"""
        # --- Depth Masking ---
        min_thresh = int(self.MIN_DEPTH_M / self.depth_scale)
        max_thresh = int(self.MAX_DEPTH_M / self.depth_scale)
        depth_mask = cv2.inRange(depth_image, min_thresh, max_thresh)
        wire_edge_mask = cv2.bitwise_and(detector_edges, depth_mask)
        
        # --- Crop Top & Bottom ---
        H, W = wire_edge_mask.shape
        wire_edge_mask[:50, :] = 0
        wire_edge_mask[-50:, :] = 0
        
        # --- Extract 3D Points from Edge Pixels ---
        points_3d = []
        for v in range(0, H, 2):
            for u in range(0, W, 2):
                if wire_edge_mask[v, u] > 0:
                    z = depth_frame.get_distance(u, v)
                    if z > 0:
                        pt = rs.rs2_deproject_pixel_to_point(intr, [u, v], z)
                        points_3d.append(pt)
        points_3d = np.array(points_3d)
        
        # Create overlay
        overlay = color_image.copy()
        
        # Add detector name to overlay
        cv2.putText(overlay, f"{detector_name}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        detection_data = None
        
        if len(points_3d) > 10:
            # Farthest X = crimp tip
            tip_idx = np.argmax(points_3d[:, 0])
            wire_tip = points_3d[tip_idx]

            # Calculate confidence
            confidence = temporal_filter.calculate_confidence(wire_tip, points_3d)
            
            # Apply temporal filtering
            filtered_tip = temporal_filter.temporal_filter(wire_tip, confidence)

            # Project to 2D
            tip_px = rs.rs2_project_point_to_pixel(intr, filtered_tip.tolist())

            # Draw with confidence-based coloring
            # Green = high confidence, Orange = medium, Red = low
            if confidence > 0.7:
                tip_color = (0, 255, 0)  # Green
            elif confidence > 0.4:
                tip_color = (0, 165, 255)  # Orange
            else:
                tip_color = (0, 0, 255)  # Red

            # Draw tip only
            cv2.circle(overlay, tuple(map(int, tip_px)), 8, tip_color, -1)
            cv2.circle(overlay, tuple(map(int, tip_px)), 12, tip_color, 2)

            # Display confidence and point count
            cv2.putText(overlay, f"Confidence: {confidence:.2f}", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(overlay, f"Points: {len(points_3d)}", (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # Display real world coordinates
            tip_real_world = filtered_tip * 1000  # Convert to millimeters
            cv2.putText(overlay, f"Tip X: {tip_real_world[0]:.1f}mm Y:{tip_real_world[1]:.1f}mm Z:{tip_real_world[2]:.1f}mm", 
                       (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            
            # Show data collection status
            with self.data_lock:
                if self.collecting_data:
                    elapsed = time.time() - self.collection_start_time
                    remaining = max(0, self.collection_duration - elapsed)
                    cv2.putText(overlay, f"COLLECTING X-COORDS: {remaining:.1f}s left", 
                               (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    cv2.putText(overlay, f"Samples collected: {len(self.collected_x_coords)}", 
                               (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # Store detection data
            detection_data = {
                'tip_px': tip_px,
                'tip_color': tip_color,
                'confidence': confidence,
                'points': len(points_3d),
                'filtered_tip': filtered_tip,
                'tip_real_world': tip_real_world
            }
            
            return overlay, confidence, len(points_3d), detection_data
        else:
            cv2.putText(overlay, "No detection", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            # Show data collection status even when no detection
            with self.data_lock:
                if self.collecting_data:
                    elapsed = time.time() - self.collection_start_time
                    remaining = max(0, self.collection_duration - elapsed)
                    cv2.putText(overlay, f"COLLECTING X-COORDS: {remaining:.1f}s left", 
                               (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    cv2.putText(overlay, f"Samples collected: {len(self.collected_x_coords)}", 
                               (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            return overlay, 0.0, 0, None
    
    def create_combined_overlay(self, color_image, log_data, sobel_data):
        """Create a combined overlay showing both LoG and Sobel detections"""
        combined = color_image.copy()
        
        # Add title
        cv2.putText(combined, "Combined: LoG + Sobel", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        # Draw LoG detection (if available)
        if log_data is not None:
            # Draw LoG detection with blue tint
            log_tip_color = (255, 0, 0)  # Blue tip
            
            cv2.circle(combined, tuple(map(int, log_data['tip_px'])), 10, log_tip_color, 2)
            cv2.circle(combined, tuple(map(int, log_data['tip_px'])), 6, log_tip_color, -1)
            
            # Label LoG detection
            cv2.putText(combined, "LoG", tuple(map(int, log_data['tip_px'] + np.array([15, -10]))), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, log_tip_color, 2)
        
        # Draw Sobel detection (if available)
        if sobel_data is not None:
            # Draw Sobel detection with green tint
            sobel_tip_color = (0, 255, 0)  # Green tip
            
            cv2.circle(combined, tuple(map(int, sobel_data['tip_px'])), 8, sobel_tip_color, -1)
            cv2.circle(combined, tuple(map(int, sobel_data['tip_px'])), 12, sobel_tip_color, 2)
            
            # Label Sobel detection
            cv2.putText(combined, "Sobel", tuple(map(int, sobel_data['tip_px'] + np.array([15, 10]))), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, sobel_tip_color, 2)
        
        # Display comparison stats
        y_pos = 60
        if log_data is not None:
            cv2.putText(combined, f"LoG - Conf: {log_data['confidence']:.2f}, Points: {log_data['points']}", 
                       (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            y_pos += 25
        
        if sobel_data is not None:
            cv2.putText(combined, f"Sobel - Conf: {sobel_data['confidence']:.2f}, Points: {sobel_data['points']}", 
                       (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            y_pos += 25
        
        # Show distance between detections if both are available
        if log_data is not None and sobel_data is not None:
            tip_distance = np.linalg.norm(log_data['filtered_tip'] - sobel_data['filtered_tip'])
            cv2.putText(combined, f"Tip distance: {tip_distance*1000:.1f}mm", 
                       (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            y_pos += 25
        
        # Display real world coordinates for the best detection
        best_data = None
        if log_data is not None and sobel_data is not None:
            # Choose the one with higher confidence
            best_data = log_data if log_data['confidence'] > sobel_data['confidence'] else sobel_data
        elif log_data is not None:
            best_data = log_data
        elif sobel_data is not None:
            best_data = sobel_data
        
        if best_data is not None:
            tip_real_world = best_data['tip_real_world']
            cv2.putText(combined, f"Best Tip X: {tip_real_world[0]:.1f}mm Y:{tip_real_world[1]:.1f}mm Z:{tip_real_world[2]:.1f}mm", 
                       (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            y_pos += 25
        
        # Show data collection status
        with self.data_lock:
            if self.collecting_data:
                elapsed = time.time() - self.collection_start_time
                remaining = max(0, self.collection_duration - elapsed)
                cv2.putText(combined, f"COLLECTING X-COORDS: {remaining:.1f}s left", 
                           (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                y_pos += 25
                cv2.putText(combined, f"Samples collected: {len(self.collected_x_coords)}", 
                           (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        return combined
    
    def destroy_node(self):
        """Clean shutdown"""
        self.detection_active = False
        if hasattr(self, 'detection_thread'):
            self.detection_thread.join(timeout=2.0)
        if hasattr(self, 'arduino') and self.arduino is not None:
            self.arduino.close()
        super().destroy_node()
    
    def run_detection(self):
        """Main detection loop running in separate thread"""
        try:
            while self.detection_active and rclpy.ok():
                # --- Get Aligned Frames ---
                frames = self.pipeline.wait_for_frames()
                aligned = self.align.process(frames)
                depth_frame = aligned.get_depth_frame()
                color_frame = aligned.get_color_frame()
                if not depth_frame or not color_frame:
                    continue

                # --- Convert to Arrays ---
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
                gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

                # --- Get intrinsics ---
                intr = depth_frame.profile.as_video_stream_profile().intrinsics
                
                # --- Edge Detection Methods ---
                blurred = cv2.GaussianBlur(gray, (5, 5), 1.5)
                
                # Method 1: Sobel Edge Detection
                sobel_x = cv2.Sobel(blurred, cv2.CV_64F, 1, 0, ksize=3)
                sobel_y = cv2.Sobel(blurred, cv2.CV_64F, 0, 1, ksize=3)
                sobel_magnitude = np.sqrt(sobel_x**2 + sobel_y**2)
                sobel_threshold = np.percentile(sobel_magnitude, 70)
                sobel_edges = (sobel_magnitude > sobel_threshold).astype(np.uint8) * 255
                
                # Method 2: Laplacian of Gaussian (LoG)
                log_blurred = cv2.GaussianBlur(gray, (5, 5), 1.4)
                laplacian = cv2.Laplacian(log_blurred, cv2.CV_64F)
                laplacian_abs = np.abs(laplacian)
                laplacian_threshold = np.percentile(laplacian_abs, 70)
                log_edges = (laplacian_abs > laplacian_threshold).astype(np.uint8) * 255
                
                # --- Process each detector individually ---
                overlay_log, conf_log, pts_log, log_data = self.process_with_detector(
                    gray, depth_image, depth_frame, intr, self.temporal_filter_log, "LoG", log_edges, color_image)
                
                overlay_sobel, conf_sobel, pts_sobel, sobel_data = self.process_with_detector(
                    gray, depth_image, depth_frame, intr, self.temporal_filter_sobel, "Sobel", sobel_edges, color_image)
                
                # --- Create Combined Overlay ---
                combined_overlay = self.create_combined_overlay(color_image, log_data, sobel_data)
                
                # Choose best detection for data collection (higher confidence)
                best_tip_x = None
                if log_data is not None and sobel_data is not None:
                    if log_data['confidence'] > sobel_data['confidence']:
                        best_tip_x = log_data['filtered_tip'][0]  # X coordinate only
                    else:
                        best_tip_x = sobel_data['filtered_tip'][0]  # X coordinate only
                elif log_data is not None:
                    best_tip_x = log_data['filtered_tip'][0]
                elif sobel_data is not None:
                    best_tip_x = sobel_data['filtered_tip'][0]
                
                # Collect X coordinate data if service is active
                with self.data_lock:
                    if self.collecting_data and best_tip_x is not None:
                        current_time = time.time()
                        if current_time - self.collection_start_time < self.collection_duration:
                            self.collected_x_coords.append(best_tip_x)
                        else:
                            # Collection period finished
                            self.collecting_data = False
                            self.collection_complete_event.set()  # Signal completion
                            self.get_logger().info(f'Finished collecting {len(self.collected_x_coords)} X coordinate samples')
                
                # --- Display Windows ---
                cv2.imshow("Combined - LoG + Sobel", combined_overlay)
                

                if cv2.waitKey(1) == 27:  # ESC to exit
                    break
                
        except Exception as e:
            self.get_logger().error(f'Detection loop error: {str(e)}')
        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = WireTipDetectorNode()
        # Now we can properly spin the node since detection runs in separate thread
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()