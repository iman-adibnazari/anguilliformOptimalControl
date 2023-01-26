# anguilliformOptimalControl
## SETUP
1. Download newest version of sofa
2. Ensure sofapython3 plugin is active
3. Setup sofapython3 to run from within python interpretter (https://sofapython3.readthedocs.io/en/latest/menu/SofaPlugin.html#within-a-python3-interpreter)
4. ensure that your execution environment defines SOFA_ROOT and PYTHONPATH correctly. This can be done on Ubuntu by adding the following to your .bashrc file
    ```    
    export SOFA_ROOT=~/PATH/TO/SOFA/INSTALLATION/DIRECTORY/ 
    export PYTHONPATH=~/PATH/TO/SOFA/INSTALLATION/DIRECTORY/plugins/SofaPython3/lib/python3/site-packages:$PYTHONPATH
    ```
5. Change name of .env_example to .env and change contained environment variables accordingly