# tool macros
CC := g++
CCFLAG := -pthread 
DBGFLAG := -g
LDLIBS := -lncurses -lsocketcan  -lmpu6050 -li2c
CCOBJFLAG := $(CCFLAG) -c 





# path macros
BIN_PATH := bin
OBJ_PATH := obj
SRC_PATH := src
INC_PATH := include
DBG_PATH := debug
LIB_PATH := lib


TARGETS := tele_keyboard write_example pid_controller write_value
TARGETS_BIN := $(addprefix $(BIN_PATH)/,$(TARGETS))

# src files & obj files
SRC := $(foreach x, $(SRC_PATH), $(wildcard $(addprefix $(x)/*,.c*)))
OBJ := $(addprefix $(OBJ_PATH)/, $(addsuffix .o, $(notdir $(basename $(SRC)))))
LIB := $(addprefix $(LIB_PATH)/, $(addsuffix .a, $(notdir $(basename $(SRC)))))
OBJ_DEBUG := $(addprefix $(DBG_PATH)/, $(addsuffix .o, $(notdir $(basename $(SRC)))))

# clean files list
DISTCLEAN_LIST := $(OBJ) \
                  $(OBJ_DEBUG) \
                  $(LIB)
CLEAN_LIST := $(DISTCLEAN_LIST)

# default rule
default: makedir all




all: $(TARGETS_BIN)



$(BIN_PATH)/tele_keyboard: $(OBJ_PATH)/tele_keyboard.o $(LIB_PATH)/libagv.a
	$(CC) $(CCFLAG) -o $@ $(OBJ_PATH)/tele_keyboard.o -L./lib -lagv $(LDLIBS)


$(BIN_PATH)/write_example: $(OBJ_PATH)/write_example.o $(LIB_PATH)/libagv.a
	$(CC) $(CCFLAG) -o $@ $(OBJ_PATH)/write_example.o -L./lib -lagv $(LDLIBS)

$(BIN_PATH)/pid_controller: $(OBJ_PATH)/pid_controller.o $(LIB_PATH)/libagv.a $(LIB_PATH)/libmpu6050.a
	$(CC) $(CCFLAG) -o $@ $(OBJ_PATH)/pid_controller.o -L./lib -lagv $(LDLIBS)

$(BIN_PATH)/write_value: $(OBJ_PATH)/write_value.o $(LIB_PATH)/libagv.a
	$(CC) $(CCFLAG) -o $@ $(OBJ_PATH)/write_value.o -L./lib -lagv $(LDLIBS)





$(OBJ_PATH)/%.o: $(SRC_PATH)/%.c* 
	$(CC) $(CCOBJFLAG)  -o $@ $< 




$(LIB_PATH)/libagv.a: $(OBJ_PATH)/AGV_Driver.o $(OBJ_PATH)/Motor_Driver_DSPIC33.o
	ar rcs $@ $^

$(LIB_PATH)/libmpu6050.a: $(OBJ_PATH)/MPU6050.o
	ar rcs $@ $^






# phony rules
.PHONY: makedir
makedir:
	@mkdir -p $(BIN_PATH) $(OBJ_PATH) $(DBG_PATH) $(LIB_PATH)


.PHONY: clean
clean:
	@echo CLEAN $(CLEAN_LIST)
	@rm -f $(CLEAN_LIST)