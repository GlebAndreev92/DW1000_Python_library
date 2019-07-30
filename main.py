"""@package main
Main module responsible for startup.

This module reads the mode variable from config and
starts the responsible module.
It also sets the logging level for the application.
"""

import logging

import config
import anchor
import tag

modes = {"tag": tag.main, "anchor": anchor.main}

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)

    try:
        modes[config.mode]()
    except KeyError:
        logging.error("Unknown mode: {}".format(config.mode))
