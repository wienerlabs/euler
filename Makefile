CC = gcc
CFLAGS = -std=c11 -Wall -Wextra -Wpedantic -Werror -I./include
CFLAGS += -Wconversion -Wsign-conversion -Wdouble-promotion
LDFLAGS = -lm -lpthread

SRC_DIR = src
TEST_DIR = tests
BUILD_DIR = build
INCLUDE_DIR = include

SRCS = $(wildcard $(SRC_DIR)/*.c)
OBJS = $(SRCS:$(SRC_DIR)/%.c=$(BUILD_DIR)/%.o)

TEST_SRCS = $(wildcard $(TEST_DIR)/*.c)
TEST_OBJS = $(TEST_SRCS:$(TEST_DIR)/%.c=$(BUILD_DIR)/%.o)

LIB = $(BUILD_DIR)/libeuler.a
TEST_BIN = $(BUILD_DIR)/euler_tests

.PHONY: all clean test lib

all: lib test

lib: $(LIB)

$(LIB): $(OBJS)
	@mkdir -p $(BUILD_DIR)
	$(AR) rcs $@ $^

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: $(TEST_DIR)/%.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

test: $(TEST_BIN)
	./$(TEST_BIN)

$(TEST_BIN): $(TEST_OBJS) $(LIB)
	$(CC) $(CFLAGS) $(TEST_OBJS) $(LIB) $(LDFLAGS) -o $@

clean:
	rm -rf $(BUILD_DIR)

format:
	clang-format -i $(SRC_DIR)/*.c $(INCLUDE_DIR)/*.h $(TEST_DIR)/*.c

analyze:
	cppcheck --enable=all --std=c11 $(SRC_DIR) $(INCLUDE_DIR)

