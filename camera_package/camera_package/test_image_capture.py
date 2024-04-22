import cv2

def main():
    # Initialize the camera capture object
    cap = cv2.VideoCapture(0)

    # Check if the camera is opened successfully
    if not cap.isOpened():
        print("Error: Unable to open camera")
        return

    try:
        # Capture frames from the camera
        while True:
            # Capture frame-by-frame
            ret, frame = cap.read()

            # Check if the frame was captured successfully
            if not ret:
                print("Error: Failed to capture frame")
                break

            # Display the captured frame
            cv2.imshow('Camera', frame)

            # Press 'q' to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Release the camera capture object
        cap.release()
        # Close all OpenCV windows
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
