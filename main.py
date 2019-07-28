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
