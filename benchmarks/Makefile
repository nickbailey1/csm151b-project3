TESTS := $(wildcard *.hex)

all:

run:
	$(foreach test, $(TESTS), timeout 10s ../tinyrv -s $(test) || exit;)

clean: