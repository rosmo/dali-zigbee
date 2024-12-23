all: fmt

.PHONY: fmt

SOURCES := $(wildcard main/*.c main/*.h main/dali/*.c main/dali/*.h)

fmt: 
	astyle \
		-n \
		--style=otbs \
		--attach-namespaces \
		--attach-classes \
		--indent=spaces=4 \
		--convert-tabs \
		--align-reference=name \
		--keep-one-line-statements \
		--pad-header \
		--pad-oper \
		--unpad-paren \
		--max-continuation-indent=120 \
		$(SOURCES)
