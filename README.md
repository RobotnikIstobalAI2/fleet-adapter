# fleet-adapter

Launch fleet-adapter.  This step adds new elements at the visualization that is started with rmf-core.

## Image build

```bash
git clone git@github.com:RobotnikIstobalAI2/fleet-adapter.git
cd fleet-adapter
git checkout humble-devel
cd container/builder
docker compose build
cd ..
```

## Launch docker with fleet-adapter

```bash
docker compose up
```

<p align="center">
  <img src="doc/fleetadapter.png" height="475" />
</p>