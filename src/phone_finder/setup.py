from setuptools import setup

package_name = 'phone_finder'

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
    maintainer='zxh0916',
    maintainer_email='zxh0916@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "draw_circle = phone_finder.draw_circle:main",
            "pid_control_test = phone_finder.pid_control_test:main",
            "find_the_phone = phone_finder.find_the_phone:main"
        ],
    },
)
