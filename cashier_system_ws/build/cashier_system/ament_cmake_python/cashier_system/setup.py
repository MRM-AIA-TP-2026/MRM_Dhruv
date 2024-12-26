from setuptools import find_packages
from setuptools import setup

setup(
    name='cashier_system',
    version='0.1.0',
    packages=find_packages(
        include=('cashier_system', 'cashier_system.*')),
)
