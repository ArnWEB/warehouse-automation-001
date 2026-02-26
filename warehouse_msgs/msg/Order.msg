# Order message for warehouse automation

string order_id           # Unique order identifier (e.g., "ORDER-001")
int32 priority            # Priority: 1=highest, 5=lowest
string[] items            # List of item SKUs
int32[] quantities        # Corresponding quantities
float64 due_time          # Unix timestamp when order is due
string status             # pending, processing, completed, cancelled
time timestamp            # When order was created
