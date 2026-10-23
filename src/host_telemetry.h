// Snapshot -> host link wire structs (lib/HostLink/src/host_messages.h).

#pragma once

#include <host_messages.h>

#include "control_task.h"

void fillTelemetry(const Snapshot& s, hostlink::Telemetry& out);
// Link health fields (rx errors) are the console's; it fills them itself.
void fillSlowStatus(const Snapshot& s, hostlink::SlowStatus& out);
