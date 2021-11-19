
from setuptools import find_packages
from setuptools import setup

setup(
      name="UTM",
      version ="1.0.0",
      author="Justin Nguyen",
      packages=['utm_monte_carlo_simulation', 'scripts'],
      #packages = find_packages(),
      #scripts=[],
      install_requires=[],
      license="MIT",
      description="Monte Carlo Traffic Simulation for UTM"
      )