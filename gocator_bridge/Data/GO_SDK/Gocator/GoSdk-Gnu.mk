ifndef verbose
	SILENT = @
endif

.PHONY: all
all: kApi GoSdk GoSystemExample 

.PHONY: kApi
kApi: 
	$(SILENT) $(MAKE) -C ../Platform/kApi -f kApi-Gnu.mk

.PHONY: GoSdk
GoSdk: kApi 
	$(SILENT) $(MAKE) -C GoSdk -f GoSdk-Gnu.mk

.PHONY: GoSystemExample
GoSystemExample: GoSdk 
	$(SILENT) $(MAKE) -C GoExamples -f GoSystemExample-Gnu.mk

.PHONY: clean
clean: kApi-clean GoSdk-clean GoSystemExample-clean 

.PHONY: kApi-clean
kApi-clean:
	$(SILENT) $(MAKE) -C ../Platform/kApi -f kApi-Gnu.mk clean

.PHONY: GoSdk-clean
GoSdk-clean:
	$(SILENT) $(MAKE) -C GoSdk -f GoSdk-Gnu.mk clean

.PHONY: GoSystemExample-clean
GoSystemExample-clean:
	$(SILENT) $(MAKE) -C GoExamples -f GoSystemExample-Gnu.mk clean


