from setuptools import find_packages, setup

package_name = 'ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'plugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hypxi',
    maintainer_email='xpercy.wu@gmail.com',
    description='User interface for FireFlight.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'ui_node = ui.ui_node:main',
        ],
    },
)
