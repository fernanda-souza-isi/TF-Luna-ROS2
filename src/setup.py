from setuptools import setup

package_name = 'tf_luna_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fernanda Souza',
    maintainer_email='fernanda@example.com',
    description='Pacote para publicar dados do TF-Luna',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = tf_luna_publisher.publisher_node:main',
            'distance_publisher = tf_luna_publisher.distance_publisher:main',
        ],
    },
)
