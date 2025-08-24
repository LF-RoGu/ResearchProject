import logging

# Configure logging
logging.basicConfig(
    filename='cumulative_tego.txt',  # log file name
    filemode='w',                    # overwrite each run
    format='%(asctime)s - %(levelname)s - %(message)s',
    level=logging.INFO
)
