from setuptools import setup,find_packages

setup(
    name='airbot_play_ik',
    version='1.0',
    packages=find_packages(),
    package_dir={"": "."},
    author='GHz',
    author_email='ghz23@mails.tsinghua.edu.cn',
    description='AIRBOT Play moveit1 ik service.',
    url='https://gitlab.com/OpenGHz/airbot_play_moveit1_base.git',
    license='MIT'
)
