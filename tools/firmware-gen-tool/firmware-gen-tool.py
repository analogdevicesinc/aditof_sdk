import json
import logging
import logging.config
import sys
import click


def setup_logging():
    with open('logger.json', 'r') as f:
        config = json.load(f)
        logging.config.dictConfig(config)


@click.command()
@click.argument('source-folder-path', type=click.Path(exists=True))
@click.argument('bin-file-name', type=click.File())
def run_gen(source_folder_path, **kwargs):
    logger = logging.getLogger(__name__)


if __name__ == "__main__":
    setup_logging()
    logger = logging.getLogger(__name__)
    run_gen()
