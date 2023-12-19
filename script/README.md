### Dependencies

This step is only needed once:

```
python3 -m venv ./venv
source ./venv/bin/activate
pip install -r requirements.txt
```

### Usage

Make sure the venv is activated before running the commands below:
```
source ./venv/bin/activate
```

See help for all command line options:

```
python3 main.py -h
```

##### Stream live data from printer

```
python3 main.py --http --live mainsailos.local
```

##### Save live data from printer to a file

```
python3 main.py --live mainsailos.local -f data.json
```

`--http` and `-f` can be combined to stream and save to a file at the same time.

##### Replay saved data from a file

```
python3 main.py --http --replay data.json
```

