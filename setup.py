from setuptools import setup, find_packages

setup(
    name="lecp6serial",
    version="1.0.0",
    description="Python wrapper for SMC LECP6 actuators",
    author="Matej Čampelj",
    author_email="matej.campelj@fs.uni-lj.si",
    packages=find_packages(),
    install_requires=[
        "pyserial",
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ]
)