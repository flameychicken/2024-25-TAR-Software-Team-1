import cv2
import cvlib as cv
from cvlib.object_detection import draw_bbox
from gtts import gTTS
from playsound import playsound
from food_facts import food_facts

class ObjectDetection:
    def __init__(self, video_source=0):
        # Initialize video capture with the default source (0 = default camera)
        self.video = cv2.VideoCapture(video_source)

    def detect_and_display(self):
        while True:
            ret, frame = self.video.read()
            if not ret:
                print("Failed to capture frame from video source.")
                break

            # Detect objects in the frame
            bbox, label, conf = cv.detect_common_objects(frame)
            output_image = draw_bbox(frame, bbox, label, conf)
            
            # Display the resulting frame with bounding boxes
            cv2.imshow("Detection", output_image)
            
            # Check if a person is detected in the frame
            if "person" in label:
                print("Person detected!")
                return True
            else:
                print("No person detected.")
                return False
            
            # Exit the loop when 'q' key is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def release_resources(self):
        # Release video capture and close windows
        self.video.release()
        cv2.destroyAllWindows()

# Example usage
if __name__ == "__main__":
    detector = ObjectDetection()
    detector.detect_and_display()
    detector.release_resources()
