from setuptools import setup

package_name = 'taeha'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SSAFY',
    maintainer_email='k4mat18@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = taeha.main:main',
            'ex_calib = taeha.ex_calib:main',
            'human_detector = taeha.human_detector:main',
            'seg_binarizer = taeha.seg_binarizer:main'
        ],
    },
)
