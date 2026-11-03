import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pkg_avant'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Exportando os arquivos de áudio mp3 para o diretório de instalação
        (os.path.join('share', package_name, 'audio'), glob(os.path.join(package_name, '*.mp3'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='avant',
    maintainer_email='joaoaug10@gmail.com',
    description='Package description',
    license='License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'simple_takeoff_land = pkg_avant.takeoff:main',
            'mission_vision_node = pkg_avant.vision:main'
        ],
    },
)
