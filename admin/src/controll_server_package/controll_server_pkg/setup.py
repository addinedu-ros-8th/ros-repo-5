from setuptools import setup, find_packages

package_name = 'controll_server_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    include_package_data=True,  # ✅ 이거 꼭 있어야 아래 package_data가 반영됨
    package_data={
        package_name: ['config/*.env'],  # ✅ .env 포함 (예: config/database.env)
    },
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jack',
    maintainer_email='khkh6380@gmail.com',
    description='Drive router node using custom MoveCommand message',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_launcher = controll_server_pkg.main_launcher:main'
        ],
    },
)
