import numpy as np

class FrameAggregator:
    def __init__(self, num_past_frames):
        self.num_past_frames = num_past_frames
        self.frames = []

    # -------------------------------
    # FUNCTION: Updating the buffer with the new frame. If buffer has more old frames than num_past frames, the oldest frame is discared.
    # -------------------------------
    def updateBuffer(self, new_frame):
        #Inserting the new frame on the top
        self.frames.insert(0, new_frame)

        #Checking if the oldest frame needs to be discarded and discarding it if needed
        if len(self.frames) > self.num_past_frames + 1:
            self.frames.pop()

    # -------------------------------
    # FUNCTION: Clearing the buffer
    # -------------------------------
    def clearBuffer(self):
        self.frames.clear()

    # -------------------------------
    # FUNCTION: Returning a list containing all points of the frames that are currently in the buffer
    # -------------------------------
    def getPoints(self):
        #Preparing a list and adding the points of all frames that are in the buffer
        points = []
        for frm in range(len(self.frames)):
            points = points + self.frames[frm]["detectedPoints"]
        
        #Returning the list
        return points