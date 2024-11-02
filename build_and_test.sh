python3 -m build
cd test
python3 -m venv venv
source venv/bin/activate
pip3 uninstall $(find ../dist -name 'pybuild_example*.whl') -y
pip3 install $(find ../dist -name 'pybuild_example*.whl') --force-reinstall
python3 test.py
deactivate