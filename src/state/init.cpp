#include "../../include/state/state.hpp"
#include <iostream>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>

Init::Init(std::shared_ptr<StreamManager> st) {
}


void Init::doAction() {
    const int fd = socket(AF_INET, SOCK_STREAM, 0);

    if (fd < 0) {
	exit(1);
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(struct sockaddr_in));
    addr.sin_family =AF_INET;
    addr.sin_port = htons(8888);
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
	exit(1);
    }

    if (listen(fd, SOMAXCONN) < 0) {
	close(fd);
	exit(1);
    }

    std::cout << "hogehoge" << std::endl;

    struct sockaddr_in get_addr;
    socklen_t len = sizeof(struct sockaddr_in);
    const int connect = accept(fd, (struct sockaddr  *)&get_addr, &len);

    if (connect < 0) {
	exit(1);
    }

    //char str
}

std::string Init::getName() {
    return "INIT";
}

State Init::getNextState() {
    return State::INIT;
}
