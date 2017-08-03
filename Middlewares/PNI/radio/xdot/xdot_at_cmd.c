#include <string.h>
#include "xdot.h"
#include "xdot_at_cmd.h"

int xdot_at_cmd_set_noargs(Xdot_AtCmdIface *iface, const char *cmd)
{
    return iface->exec(iface->buffer, snprintf(iface->buffer, iface->size,
            "%s\r\n", cmd));
}

int xdot_at_cmd_set_int(Xdot_AtCmdIface *iface, const char *cmd,
        int value)
{
    return iface->exec(iface->buffer, snprintf(iface->buffer, iface->size,
            "%s=%d\r\n", cmd, value));
}

int xdot_at_cmd_set_str(Xdot_AtCmdIface *iface, const char *cmd,
        const char *value)
{
    return iface->exec(iface->buffer, snprintf(iface->buffer, iface->size,
            "%s=%s\r\n", cmd, value));
}

int xdot_at_cmd_set_int_ne(Xdot_AtCmdIface *iface, const char *cmd,
        int value)
{
    return iface->exec(iface->buffer, snprintf(iface->buffer, iface->size,
            "%s%d\r\n", cmd, value));
}

int xdot_at_cmd_set_int_w_str(Xdot_AtCmdIface *iface, const char *cmd,
        const int id, const char *s)
{
    return iface->exec(iface->buffer, snprintf(iface->buffer, iface->size,
            "%s=%d,%s\r\n", cmd, id, s));
}

int xdot_at_cmd_get(Xdot_AtCmdIface *iface, const char *cmd)
{
    return iface->exec(iface->buffer, snprintf(iface->buffer, iface->size,
            "%s?\r\n", cmd));
}

