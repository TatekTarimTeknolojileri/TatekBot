from setuptools import setup

package_name = 'tatekbot_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[
        'commander'
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Package for controlling a 4WD robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'commander = tatekbot_control.commander:main'
        ],
    },
)
