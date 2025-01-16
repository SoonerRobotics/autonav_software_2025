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
            'playback = autonav_playback.playback:main',
            'dev_vid = autonav_playback.dev_vid:main',
            'tester = autonav_playback.tester:main',
            'combiner = autonav_playback.CombinedView:main',
        ],
    },
)
