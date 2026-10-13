#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/tool.hpp>
#include <iostream>

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cerr << "usage: tool_list_probe <robot_serial>\n";
        return 1;
    }
    flexiv::rdk::Robot robot(argv[1]);
    flexiv::rdk::Tool tool(robot);

    std::cout << "current tool: " << tool.name() << "\n\n";

    auto names = tool.list();
    std::cout << "configured tools (" << names.size() << "):\n";
    for (const auto& n : names) {
        std::cout << "  - " << n;
        auto p = tool.params(n);
        std::cout << "  (mass=" << p.mass << " kg)\n";
    }
    return 0;
}
