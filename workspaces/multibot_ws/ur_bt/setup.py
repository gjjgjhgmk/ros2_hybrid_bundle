#!/usr/bin/env python3
"""
PARM BT 安装脚本
"""

from pathlib import Path

from setuptools import setup, find_packages

PACKAGE_ROOT = Path(__file__).resolve().parent

with (PACKAGE_ROOT / "README.md").open("r", encoding="utf-8") as fh:
    long_description = fh.read()

with (PACKAGE_ROOT / "requirements.txt").open("r", encoding="utf-8") as fh:
    requirements = [line.strip() for line in fh if line.strip() and not line.startswith("#")]

setup(
    name="ur-bt",
    version="0.1.0",
    author="PARM Team",
    author_email="parm@example.com",
    description="基于py_trees的机器人决策系统框架",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/parm-team/ur-bt",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ],
    python_requires=">=3.8",
    install_requires=requirements,
    extras_require={
        "dev": [
            "pytest>=7.0.0",
            "pytest-cov>=4.0.0",
            "black>=22.0.0",
            "flake8>=5.0.0",
            "mypy>=1.0.0",
        ],
    },
    entry_points={
        "console_scripts": [
            "ur-bt=ur_bt.cli:main",
        ],
    },
    include_package_data=True,
    package_data={
        "ur_bt": ["config.yaml"],
    },
)
