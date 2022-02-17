DATA_NAME:=monaco
OSRM_BUILD_DIR?=../../build
PROFILE_ROOT:=../../profiles
SCRIPT_ROOT:=../../scripts
OSRM_EXTRACT:=$(OSRM_BUILD_DIR)/osrm-extract
OSRM_CONTRACT:=$(OSRM_BUILD_DIR)/osrm-contract
OSRM_PARTITION:=$(OSRM_BUILD_DIR)/osrm-partition
OSRM_CUSTOMIZE:=$(OSRM_BUILD_DIR)/osrm-customize
OSRM_ROUTED:=$(OSRM_BUILD_DIR)/osrm-routed
POLY2REQ:=$(SCRIPT_ROOT)/poly2req.js
MD5SUM:=$(SCRIPT_ROOT)/md5sum.js
TIMER:=$(SCRIPT_ROOT)/timer.js
PROFILE:=$(PROFILE_ROOT)/car.lua

all: data

<<<<<<< HEAD
data: ch/$(DATA_NAME).osrm.hsgr corech/$(DATA_NAME).osrm.hsgr mld/$(DATA_NAME).osrm.partition

clean:
	-rm -r $(DATA_NAME).*
	-rm -r ch corech mld
=======
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 1b41e999e (Implementing algorithms to perform routing for electric vehicles.)
=======
>>>>>>> 18d37b1a2 (Implementing algorithms to perform routing for electric vehicles.)
data: ch/$(DATA_NAME).osrm.hsgr corech/$(DATA_NAME).osrm.hsgr street/street.osrm.hsgr
#mld/$(DATA_NAME).osrm.partition

clean:
	-rm -rf $(DATA_NAME).osrm*
	-rm -rf street.osrm*
	-rm -rf ch corech mld street

$(DATA_NAME).osrm: $(DATA_NAME).osm.pbf $(DATA_NAME).poly $(PROFILE) $(OSRM_EXTRACT)
	@echo "Verifiyng data file integrity..."
	$(MD5SUM) -c data.md5sum
	@echo "Running osrm-extract..."
	$(TIMER) "osrm-elevation\t$@" $(OSRM_ELEVATION) $(DATA_NAME).osm.pbf --geotiff elevation/monaco_south.tif --geotiff elevation/monaco_north.tif
	$(TIMER) "osrm-extract\t$@" $(OSRM_EXTRACT) $< -p $(PROFILE)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
data: ch/$(DATA_NAME).osrm.hsgr corech/$(DATA_NAME).osrm.hsgr mld/$(DATA_NAME).osrm.partition street/street.osrm.hsgr
=======
data: ch/$(DATA_NAME).osrm.hsgr corech/$(DATA_NAME).osrm.hsgr street/street.osrm.hsgr mld/$(DATA_NAME).osrm.partition
>>>>>>> 8bff31d5b (Sehr wichtiger Commit; Ich habe die zentralen Bugs der Verbrauchsberechnung gefunden; NON_ZERO_CONSUMPTION als define eingefügt; kleinzeug)
=======
data: ch/$(DATA_NAME).osrm.hsgr corech/$(DATA_NAME).osrm.hsgr street/street.osrm.hsgr
#mld/$(DATA_NAME).osrm.partition
>>>>>>> 734713053 (Erste Version von DBSCan in C++)

clean:
	-rm -r $(DATA_NAME).osrm*
	-rm -r street.osrm.*
	-rm -r ch corech mld street
>>>>>>> bf370cdff (WIP: Code zum erstellen des Charger Graphen hinzugefuegt)
=======
>>>>>>> 99695d3a2 (DBScan neu implementiert (erfolgreich))
=======
>>>>>>> 1b41e999e (Implementing algorithms to perform routing for electric vehicles.)
=======
>>>>>>> 18d37b1a2 (Implementing algorithms to perform routing for electric vehicles.)
>>>>>>> 50f7a6754 (Split consumption into two factors)

ch/$(DATA_NAME).osrm: $(DATA_NAME).osrm
	mkdir -p ch
	cp $(DATA_NAME).osrm $(DATA_NAME).osrm.* ch/

corech/$(DATA_NAME).osrm: $(DATA_NAME).osrm
	mkdir -p corech
	cp $(DATA_NAME).osrm $(DATA_NAME).osrm.* corech/

mld/$(DATA_NAME).osrm: $(DATA_NAME).osrm
	mkdir -p mld
	cp $(DATA_NAME).osrm $(DATA_NAME).osrm.* mld/

<<<<<<< HEAD
=======
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 50f7a6754 (Split consumption into two factors)
$(DATA_NAME).osrm: $(DATA_NAME).osm.pbf $(DATA_NAME).poly $(PROFILE) $(OSRM_EXTRACT)
	@echo "Verifiyng data file integrity..."
	$(MD5SUM) -c data.md5sum
	@echo "Running osrm-extract..."
<<<<<<< HEAD
	$(TIMER) "osrm-extract\t$@" $(OSRM_EXTRACT) $< -p $(PROFILE)
=======
	$(TIMER) "osrm-elevation\t$@" $(OSRM_ELEVATION) $(DATA_NAME).osm.pbf --geotiff elevation/monaco_south.tif --geotiff elevation/monaco_north.tif
	$(TIMER) "osrm-extract\t$@" $(OSRM_EXTRACT) $< -p $(PROFILE)
>>>>>>> bf370cdff (WIP: Code zum erstellen des Charger Graphen hinzugefuegt)
=======
>>>>>>> 99695d3a2 (DBScan neu implementiert (erfolgreich))
=======
>>>>>>> 1b41e999e (Implementing algorithms to perform routing for electric vehicles.)
=======
>>>>>>> 18d37b1a2 (Implementing algorithms to perform routing for electric vehicles.)
>>>>>>> 50f7a6754 (Split consumption into two factors)

ch/$(DATA_NAME).osrm.hsgr: ch/$(DATA_NAME).osrm $(PROFILE) $(OSRM_CONTRACT)
	@echo "Running osrm-contract..."
	$(TIMER) "osrm-contract\t$@" $(OSRM_CONTRACT) $<

corech/$(DATA_NAME).osrm.hsgr: corech/$(DATA_NAME).osrm $(PROFILE) $(OSRM_CONTRACT)
	@echo "Running osrm-contract..."
	$(TIMER) "osrm-contract\t$@" $(OSRM_CONTRACT) --core=0.5 $<

mld/$(DATA_NAME).osrm.partition: mld/$(DATA_NAME).osrm $(PROFILE) $(OSRM_PARTITION)
	@echo "Running osrm-partition..."
	$(TIMER) "osrm-partition\t$@" $(OSRM_PARTITION) $<
	$(TIMER) "osrm-customize\t$@" $(OSRM_CUSTOMIZE) $<

$(DATA_NAME).requests: $(DATA_NAME).poly
	$(POLY2REQ) $(DATA_NAME).poly > $(DATA_NAME).requests

benchmark: data $(DATA_NAME).requests
	@echo "Running benchmark..."
	@/bin/sh -c '$(OSRM_ROUTED) --algorithm=CH ch/$(DATA_NAME).osrm > /dev/null & echo "$$!" > osrm-routed.pid'
	@sleep 1
	$(TIMER) "queries\tCH" "cat $(DATA_NAME).requests | xargs curl &> /dev/null"
	@cat osrm-routed.pid | xargs kill
	@rm osrm-routed.pid
	@/bin/sh -c '$(OSRM_ROUTED) --algorithm=CoreCH corech/$(DATA_NAME).osrm > /dev/null & echo "$$!" > osrm-routed.pid'
	@sleep 1
	$(TIMER) "queries\tCoreCH" "cat $(DATA_NAME).requests | xargs curl &> /dev/null"
	@cat osrm-routed.pid | xargs kill
	@rm osrm-routed.pid
	@/bin/sh -c '$(OSRM_ROUTED) --algorithm=MLD mld/$(DATA_NAME).osrm > /dev/null & echo "$$!" > osrm-routed.pid'
	@sleep 1
	$(TIMER) "queries\tMLD" "cat $(DATA_NAME).requests | xargs curl &> /dev/null"
	@cat osrm-routed.pid | xargs kill
	@rm osrm-routed.pid
	@echo "**** timings ***"
	@cat /tmp/osrm.timings
	@echo "****************"

checksum:
	$(MD5SUM) $(DATA_NAME).osm.pbf $(DATA_NAME).poly > data.md5sum

.PHONY: clean checksum benchmark data
