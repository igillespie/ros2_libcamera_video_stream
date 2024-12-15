import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from sensor_msgs.msg import Image, CompressedImage  # Import both message types
from cv_bridge import CvBridge  # type: ignore
import subprocess
import cv2  # type: ignore
import numpy as np  # type: ignore


class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher_node')

        # Declare parameters for codec, dimensions, and FPS
        self.declare_parameter('codec', 'mjpeg')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)

        # Retrieve parameter values
        self.codec = self.get_parameter('codec').get_parameter_value().string_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value

        self.get_logger().info(f"Starting video stream with codec={self.codec}, width={self.width}, height={self.height}, fps={self.fps}")

        # Start the video capture process
        self.pipeline = subprocess.Popen(
            [
                'libcamera-vid',
                '--codec', self.codec,
                '--width', str(self.width),
                '--height', str(self.height),
                '--framerate', str(self.fps),
                '--nopreview',
                '--timeout', '0',  # Run indefinitely
                '--output', '-'
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        # Create publishers for raw and compressed image messages
        self.raw_publisher = self.create_publisher(Image, 'video_frames', 10)
        self.compressed_publisher = self.create_publisher(CompressedImage, 'video_frames/compressed', 10)

        # Bridge for ROS image messages
        self.bridge = CvBridge()
        self.buffer = b''  # Buffer for storing MJPEG data
        self.timer = self.create_timer(1 / self.fps, self.process_frames)

    def process_frames(self):
        try:
            # Read MJPEG data from the pipeline
            data = self.pipeline.stdout.read(65536)  # Read 64 KB chunks
            self.buffer += data

            # Process complete JPEG frames
            while b'\xFF\xD8' in self.buffer and b'\xFF\xD9' in self.buffer:
                start = self.buffer.find(b'\xFF\xD8')  # Start of Image (SOI)
                end = self.buffer.find(b'\xFF\xD9') + 2  # End of Image (EOI)

                # Extract the JPEG frame
                jpeg_frame = self.buffer[start:end]
                self.buffer = self.buffer[end:]  # Remove processed frame from the buffer

                # Decode JPEG to OpenCV format (raw image)
                frame_array = np.frombuffer(jpeg_frame, dtype=np.uint8)
                frame = cv2.imdecode(frame_array, cv2.IMREAD_COLOR)

                # Publish raw image
                raw_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                raw_msg.header.stamp = self.get_clock().now().to_msg()
                self.raw_publisher.publish(raw_msg)

                # Publish compressed image
                compressed_msg = CompressedImage()
                compressed_msg.header.stamp = self.get_clock().now().to_msg()
                compressed_msg.format = "jpeg"
                compressed_msg.data = jpeg_frame
                self.compressed_publisher.publish(compressed_msg)

                self.get_logger().debug("Published raw and compressed frames")

        except Exception as e:
            self.get_logger().error(f"Error processing frames: {e}")

    def destroy_node(self):
        if self.pipeline.poll() is None:
            self.pipeline.terminate()
            self.pipeline.wait()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()