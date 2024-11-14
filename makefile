# o3 RISC simulator
#
# makefile
#
# Lukas Heine 2021

target	=	o3
outfile	= 	$(target).x

cc 	=	g++-11
ccflags	=	-Ofast -g -MMD -std=c++20 -Wall -Wextra -masm=intel
ldflags	=	-L /usr/local/lib -lgtest -lgtest_main -pthread

# folders
sdir 	= 	src/
cdir 	= 	$(sdir)core/
fdir 	= 	$(sdir)frontend/
tdir 	= 	tests/

# sources
ssrc 	= 	$(wildcard $(sdir)*.cc)
csrc 	= 	$(wildcard $(cdir)*.cc)
fsrc 	= 	$(wildcard $(fdir)*.cc)
tsrc 	= 	$(wildcard $(tdir)*.cc)

# objects
sobs 	= 	$(patsubst %.cc, %.o, $(ssrc))
cobs 	= 	$(patsubst %.cc, %.o, $(csrc))
fobs 	= 	$(patsubst %.cc, %.o, $(fsrc))
tobs 	= 	$(patsubst %.cc, %.o, $(tsrc))

# dependencies
sdeps	=	$(patsubst %.cc, %.d, $(ssrc))
cdeps	=	$(patsubst %.cc, %.d, $(csrc))
fdeps	=	$(patsubst %.cc, %.d, $(fsrc))
tdeps	=	$(patsubst %.cc, %.d, $(tsrc))


.PHONY: all test clean run re nolog icpc


all: $(cobs) $(fobs) $(sobs) $(outfile)

# c++20 machine broke
# icpc: cc 		= 	icpc
# icpc: ccflags	+= 	-diag-disable=11074,11076
# icpc: all


re: clean all


$(sobs): $(sdir)%.o: $(sdir)%.cc
	$(cc) $(ccflags) -o $@ -c $<

$(cobs): $(cdir)%.o: $(cdir)%.cc
	$(cc) $(ccflags) -o $@ -c $<

$(fobs): $(fdir)%.o: $(fdir)%.cc
	$(cc) $(ccflags) -o $@ -c $<

# todo may need lmath later on
$(outfile): $(sobs) $(cobs) $(fobs)
	$(cc) $(ccflags) -o $@ $^


# always recompile main to link gtest
test: $(sdir)$(target).cc $(filter-out $(sdir)$(target).o, $(sobs)) $(cobs) $(fobs) $(tobs)
	$(cc) $(ccflags) -D simtest -o gtest_$(outfile) $^ -I /usr/local/include/gtest $(ldflags)

$(tobs): $(tdir)%.o: $(tdir)%.cc
	$(cc) $(ccflags) -D simtest -o $@ -c $<


nolog: ccflags += -Dnolog -DNDEBUG
nolog: all


run1:
	./$(outfile) -v -m 00000000111111112222222233333333


run4:
	./$(outfile) -v -m 0000000011111111222222223333333344444444555555556666666677777777\
	8888888899999999aaaaaaaabbbbbbbbccccccccddddddddeeeeeeeeffffffff


clean:
	rm -f $(outfile) gtest_$(outfile)
	rm -f $(sdir)*.o $(cdir)*.o $(fdir)*.o $(tdir)*.o
	rm -f $(sdir)*.d $(cdir)*.d $(fdir)*.d $(tdir)*.d


-include $(sdeps)
-include $(cdeps)
-include $(fdeps)
-include $(tdeps)
