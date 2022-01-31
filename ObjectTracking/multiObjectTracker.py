import cv2
import sys


def drawBoundingBox(bbox):
    # Tracking success
    p1 = (int(bbox[0]), int(bbox[1]))
    p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
    # cv2.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)
    x, y = p1[0], p1[1]
    w, h = p2[0] - p1[0], p2[1] - p1[1]
    ROI = frame[y:y + h, x:x + w]
    blur = cv2.GaussianBlur(ROI, (347, 347), 0)
    frame[y:y + h, x:x + w] = blur


# Read video
video = cv2.VideoCapture("highway.mp4") # for using an input video
#video = cv2.VideoCapture(0, cv2.CAP_DSHOW) # for using CAM

# Exit if video not opened.
if not video.isOpened():
  print("Could not open video")
  sys.exit()

# Read first frame.
ok, frame = video.read()
if not ok:
  print ('Cannot read video file')
  sys.exit()


#cv2.imshow("Frame", frame)


# Define an initial bounding box
#bbox = (287, 23, 86, 320)


## Select boxes
bboxes = []


# OpenCV's selectROI function doesn't work for selecting multiple objects in Python
# So we will call this function in a loop till we are done selecting all objects
while True:
  # draw bounding boxes over objects
  # selectROI's default behaviour is to draw box starting from the center
  # when fromCenter is set to false, you can draw box starting from top left corner
  bbox = cv2.selectROI('MultiTracker', frame)
  bboxes.append(bbox)
  print("Press q to quit selecting boxes and start tracking")
  print("Press any other key to select next object")
  k = cv2.waitKey(0) & 0xFF
  if (k == 113):  # q is pressed
    break

print('Selected bounding boxes {}'.format(bboxes))

# for information only (l'ho messo solo per mostrare a video il tracker in uso, andrebbe fatto come nel file objectTrackingBlurring.py)
tracker_type = "MOSSE"

# Create MultiTracker object
multiTracker = cv2.MultiTracker_create()

# Initialize MultiTracker
for bbox in bboxes:
  # PER CAMBIARE TRACKER SCRIVERE cv2.Tracker<NOME_TRACKER>_create() i nome dei tracker sono in objectTrackingBlurring.py
  multiTracker.add(cv2.TrackerMOSSE_create(), frame, bbox)

while video.isOpened():
    success, frame = video.read()

    if not success:
        break

    # start timer
    timer = cv2.getTickCount()

    # get updated location of objects in subsequent frames
    success, boxes = multiTracker.update(frame)

    # Calculate Frames per second (FPS)
    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);

    # draw tracked objects
    for i, newbox in enumerate(boxes):
        if success:
            drawBoundingBox(newbox)
        else:
            # Tracking failure
            cv2.putText(frame, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

    # Display tracker type on frame
    cv2.putText(frame, tracker_type + " Tracker", (100, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2);

    # Display FPS on frame
    cv2.putText(frame, "FPS : " + str(int(fps)), (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2);
       
    # show frame
    cv2.imshow("MultiTracker", frame)

    # Exit if ESC pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):  # if press SPACE bar
        break

video.release()
cv2.destroyAllWindows()


