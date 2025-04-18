"""!
@file frameAggregator.py
@brief Handles frame aggregation for radar data processing.

@details This module maintains a buffer of past radar frames, allowing access to historical data
for filtering, clustering, and object detection. It enables efficient management of detected
points across multiple frames to improve tracking performance.

@defgroup Frame_Aggregator Frame Aggregator
@brief Manages frame buffering and point retrieval for radar processing.
@{
"""

## @defgroup Frame_Aggregator Frame Aggregator
## @brief Manages frame buffering and point retrieval for radar processing.
## @{
class FrameAggregator:
    """!
    @class FrameAggregator
    @ingroup Frame_Aggregator
    @brief Maintains a rolling buffer of radar frames.
    
    @details This class stores a configurable number of past frames to enable data persistence
    and retrieval for multi-frame processing. It supports buffer updates, clearing, and
    retrieving all detected points from stored frames.
    """
    def __init__(self, num_past_frames):
        """!
        @brief Initializes the frame buffer.
        @param in num_past_frames The number of historical frames to maintain in the buffer.
        
        @ingroup Frame_Aggregator
        """
        self.num_past_frames = num_past_frames
        self.frames = []


    def updateBuffer(self, new_frame):
        """!
        @brief Updates the frame buffer with a new radar frame.
        @param in new_frame The latest radar frame containing detected points.
        
        @ingroup Frame_Aggregator
        """
        #Inserting the new frame on the top
        self.frames.insert(0, new_frame)

        #Checking if the oldest frame needs to be discarded and discarding it if needed
        if len(self.frames) > self.num_past_frames + 1:
            self.frames.pop()

    # -------------------------------
    # FUNCTION: Clearing the buffer
    # -------------------------------
    def clearBuffer(self):
        """!
        @brief Clears the frame buffer.
                
        @ingroup Frame_Aggregator
        """
        self.frames.clear()

    # -------------------------------
    # FUNCTION: Returning a list containing all points of the frames that are currently in the buffer
    # -------------------------------
    def getPoints(self):
        """!
        @brief Retrieves all detected points from stored frames.
        
        @return List of detected points from all stored frames.
        
        @ingroup Frame_Aggregator
        """
        #Preparing a list and adding the points of all frames that are in the buffer
        points = []
        for frm in range(len(self.frames)):
            points = points + self.frames[frm]["detectedPoints"]
        
        #Returning the list
        return points
    
## @}  # End of Frame_Aggregator group