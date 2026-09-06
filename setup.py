from setuptools import setup

# All packaging metadata lives in pyproject.toml. setuptools reads it and it
# overrides anything declared here, so declaring it twice only lets the two
# drift apart - which is exactly how the version numbers came to disagree.
# This shim exists because the publish workflow calls 'python setup.py sdist'.
setup()
