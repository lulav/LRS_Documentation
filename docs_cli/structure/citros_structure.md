# Citros Repository File Structure

The following folder and file structure is automatically generated for you (when you run `citros init`):

- .citros
  - notebooks
  - parameter_setups
    - functions
      - `my_func.py`
    - `default_param_setup.json`
  - reports
  - [runs](#directory-runs)
  - simulations
    - `simulation_foo.json`
    - `simulation_bar.json`
  - workflows
    - `default_flow.json`
  - `citros_repo_id`
  - `project.json`
  - `settings.json`
  - `user_commit`

## Directory `runs`

The `runs` directory will be populated with further files and folder every time you run a simulation (via `citros run`):

- runs
  - simulation name
    - batch name
      - run id
        - bag
          - `bag_0.db3`
          - `metadata.yaml`
        - config
          - `pkg1.yaml`
          - `pkg2.yaml`
        - msgs
          - `pkg1`
            - `msg`
              - `foo.msg`
          - `pkg2`
            - `msg`
              - `bar.msg`
        - `citros.log`
        - `environment.json`
        - `info.json`
        - `metrics.csv`
        - `otlp_trace`
        - `ros.log`
      - `info.json`


# Directory `Notebooks`
