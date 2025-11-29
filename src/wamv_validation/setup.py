from setuptools import setup

package_name = 'wamv_validation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='Thomas Istvan',
    maintainer_email='you@example.com',
    description='VRX/WAM-V validation tools',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wamv_twist_from_pose = wamv_validation.wamv_twist_from_pose:main',
        ],
    },
)
