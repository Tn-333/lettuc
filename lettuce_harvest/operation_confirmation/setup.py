from setuptools import setup

package_name = 'operation_confirmation'

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
    maintainer='tone',
    maintainer_email='tone@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_pro_test_node = operation_confirmation.image_pro_test_node:main',
            'publish_for_set_origin = operation_confirmation.publish_for_arm:main',
        ],
    },
)
