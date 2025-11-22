"""Système de logging pour Eurobot 2026."""

import logging
import sys
from datetime import datetime


class Logger:
    """Gestionnaire de logs centralisé."""
    
    _instance = None
    _logger = None
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(Logger, cls).__new__(cls)
            cls._instance._init_logger()
        return cls._instance
    
    @classmethod
    def _init_logger(cls):
        """Initialise le logger."""
        cls._logger = logging.getLogger('Eurobot2026')
        cls._logger.setLevel(logging.DEBUG)
        
        # Console handler
        ch = logging.StreamHandler(sys.stdout)
        ch.setLevel(logging.DEBUG)
        
        # Formatter
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        ch.setFormatter(formatter)
        cls._logger.addHandler(ch)
    
    @staticmethod
    def debug(msg):
        """Log debug."""
        Logger._logger.debug(msg)
    
    @staticmethod
    def info(msg):
        """Log info."""
        Logger._logger.info(msg)
    
    @staticmethod
    def warning(msg):
        """Log warning."""
        Logger._logger.warning(msg)
    
    @staticmethod
    def error(msg):
        """Log error."""
        Logger._logger.error(msg)
