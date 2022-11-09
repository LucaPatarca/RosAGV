import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import ament_index_python
import os.path
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
import requests

SCALE = 0.00392
RES_DIR = os.path.join(
    ament_index_python.get_package_share_directory('object_detection'),
    'resource'
)
NET_CONF_PATH = os.path.join(RES_DIR, 'yolov3.cfg')
NET_CLASS_PATH = os.path.join(RES_DIR, 'yolov3.txt')
NET_WEIGHTS_PATH = os.path.join(RES_DIR, 'yolov3.weights')

def get_output_layers(net):
    layer_names = net.getLayerNames()
    try:
        output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
    except:
        output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    return output_layers


class ImageRecognitionNode(Node):

    def __init__(self):
        super().__init__('object_detection')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',
            self.image_callback,
            1)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(
            Detection2DArray,
            '/agv/detections',
            1
        )
        # load network classes
        with open(NET_CLASS_PATH, 'r') as f:
            self.classes = [line.strip() for line in f.readlines()]
        # initialize object type colors
        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))
        if not os.path.exists(NET_WEIGHTS_PATH):
            self.download_weights()
        self.net = cv2.dnn.readNet(NET_WEIGHTS_PATH, NET_CONF_PATH)

    def download_weights(self):
        self.get_logger().info("Downloading pre-trained neural network...")
        res = requests.get("https://pjreddie.com/media/files/yolov3.weights")
        with open(NET_WEIGHTS_PATH, "wb") as f:
            f.write(res.content)
        self.get_logger().info("Done.")

    def get_detections(self, outs, image) -> Detection2DArray:
        Width = image.shape[1]
        Height = image.shape[0]

        class_ids = []
        confidences = []
        boxes = []
        conf_threshold = 0.5
        nms_threshold = 0.4

        detections = Detection2DArray()

        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > conf_threshold:
                    center_x = int(detection[0] * Width)
                    center_y = int(detection[1] * Height)
                    w = int(detection[2] * Width)
                    h = int(detection[3] * Height)
                    x = center_x - w / 2
                    y = center_y - h / 2
                    class_ids.append(class_id)
                    confidences.append(float(confidence))
                    boxes.append([x, y, w, h])


        indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)

        for i in indices:
            try:
                box = boxes[i]
            except:
                i = i[0]
                box = boxes[i]
            
            detection = Detection2D()
            detection.bbox.size_x = float(box[2])
            detection.bbox.size_y = float(box[3])
            detection.bbox.center.x = box[0]
            detection.bbox.center.y = box[1]
            result = ObjectHypothesisWithPose()
            result.id = str(self.classes[class_ids[i]])
            result.score = confidences[i]
            detection.results.append(result)
            detections.detections.append(detection)
        
        return detections

    def image_callback(self, msg: CompressedImage):
        image = np.asarray(bytearray(msg.data), dtype="uint8")
        image = cv2.imdecode(image, cv2.IMREAD_COLOR)
        blob = cv2.dnn.blobFromImage(image, SCALE, (416,416), (0,0,0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(get_output_layers(self.net))
        detections = self.get_detections(outs, image)
        self.publisher.publish(detections)


def main(args=None):
    rclpy.init(args=args)

    node = ImageRecognitionNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()