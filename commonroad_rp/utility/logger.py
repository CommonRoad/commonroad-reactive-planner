import logging
import os
from datetime import datetime
import sys


def initialize_logger(config) -> logging.Logger:
    """
    Initializes the logging module and returns a logger.
    """
    # create log directory
    os.makedirs(config.general.path_logs, exist_ok=True)

    # create logger
    logger = logging.getLogger()

    # create file handler (outputs to file)
    string_date_time = datetime.now().strftime("_%Y_%m_%d_%H-%M-%S")
    path_log = os.path.join(config.general.path_logs, f"{config.name_scenario}{string_date_time}.log")
    file_handler = logging.FileHandler(path_log)

    # set logging levels
    logger.setLevel(logging.DEBUG)
    file_handler.setLevel(logging.DEBUG)

    # create log formatter
    # formatter = logging.Formatter('%(asctime)s\t%(filename)s\t\t%(funcName)s@%(lineno)d\t%(levelname)s\t%(message)s')
    log_formatter = logging.Formatter("%(levelname)-8s [%(asctime)s] --- %(message)s (%(filename)s:%(lineno)s)",
                                  "%Y-%m-%d %H:%M:%S")
    file_handler.setFormatter(log_formatter)

    # create stream handler (prints to stdout)
    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setLevel(logging.DEBUG)

    # create stream formatter
    stream_formatter = logging.Formatter("%(levelname)-8s [ReactivePlanner]: %(message)s")
    stream_handler.setFormatter(stream_formatter)

    # add handlers
    logger.addHandler(file_handler)
    logger.addHandler(stream_handler)

    return logger
