from setuptools import setup

package_name = 'uwb_distance'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'uwb_distance.uwb_receiver'
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uwb_receiver = uwb_distance.uwb_receiver:main'
        ],
    },
)

