from distutils.core import setup, Extension

setup(name = "Camera",
      version = "1.0",
      ext_modules = [Extension("camera", ["Camera.cpp"])])