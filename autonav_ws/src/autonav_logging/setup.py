from setuptools import find_packages, setup

package_name = 'autonav_logging'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='ghair244@gmail.com',
    description='TODO: Logging Package 🤩',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spam = autonav_logging.testTopicsSpam:main',
            'talker = autonav_logging.topic_pub:main',
        ],
    },
)
