"""
Módulo de controladores para robots.
"""

from .navigation import NavigationController
from .random_navigation import RandomNavigationController

__all__ = [
    'NavigationController',
    'RandomNavigationController'
]
