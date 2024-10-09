from setuptools import find_packages, setup

package_name = 'autonav_playback'

setup(
    name=package_name,
    version='0.0.0',
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
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spam = autonav_playback.testTopicsSpam:main',
            'master = autonav_playback.topic_pub:main',
            'playback = autonav_playback.playback:main',
        ],
    },
)
