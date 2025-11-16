PY=python

setup:
	@$(PY) -m pip install -r requirements.txt

run:
	@$(PY) -m src.run_scenario --config configs/default.yaml

run-small:
	@$(PY) -m src.run_scenario --config configs/default.yaml --scenario data/scenarios/case_small.yaml

test:
	@pytest -q

clean:
	@rm -f reports/*.csv reports/*.json reports/*.png
