#!/usr/bin/env python3

from setuptools import setup

with open('../README.md') as f:
    long_description = f.read()


setup(
    name='telemetrix-rpi-pico-w',
    packages=['telemetrix_rpi_pico_w', 'telemetrix_rpi_pico_w_aio'],
    install_requires=['pyserial'],

    version='1.0',
    description="Remotely Control And Monitor A Raspberry Pi Pico W",
    long_description=long_description,
    long_description_content_type='text/markdown',

    author='Alan Yorinks',
    author_email='MisterYsLab@gmail.com',
    url='https://github.com/MrYsLab/telemetrix-rpi-pico-w',
    download_url='https://github.com/MrYsLab/telemetrix-rpi-pico-w',
    keywords=['telemetrix', 'Raspberry_Pi_Pico_W', 'Protocol', 'Python'],
    classifiers=[
        'Development Status :: 5 - Production/Stable',
        'Environment :: Other Environment',
        'Intended Audience :: Developers',
        'Intended Audience :: Education',
        'License :: OSI Approved :: GNU Affero General Public License v3 or later (AGPLv3+)',
        'Operating System :: OS Independent',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Topic :: Software Development :: Libraries :: Python Modules'
    ],
)
