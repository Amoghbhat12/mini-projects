module elevator_control (
    input wire clk,                  // Clock signal
    input wire reset,                // Reset signal
    input wire [3:0] floor_req,      // Floor requests (4 bits for 4 floors)
    input wire [3:0] current_floor,  // Current floor (4 bits)
    output reg [3:0] next_floor,     // Next floor to move to
    output reg motor_up,              // Motor control for moving up
    output reg motor_down,            // Motor control for moving down
    output reg door_open,              // Control signal for door
    output reg [3:0] timer_count       // Timer count output
);

    // Internal signals
    reg [3:0] floor_status; // To hold which floor the elevator is at
    wire [3:0] request;     // Current requests
    wire [3:0] comparator_out; // Comparator output
    reg [3:0] state;         // State variable for FSM
    reg [7:0] timer;         // Timer counter for timing operations

    // States for FSM
    parameter IDLE = 4'b0000, MOVE_UP = 4'b0001, MOVE_DOWN = 4'b0010, OPEN_DOOR = 4'b0011, CLOSE_DOOR = 4'b0100;

    // JK Flip-Flops
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            floor_status <= 4'b0000; // Reset the floor status
        end else begin
            floor_status <= floor_req; // Store current floor requests
        end
    end

    // 4-bit Comparator to check if the requested floor is the same as the current floor
    assign comparator_out = current_floor - floor_req;

    // Next floor logic and state transitions
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            next_floor <= 4'b0000;  // Reset next floor
            state <= IDLE;           // Start in idle state
            timer <= 8'b00000000;    // Reset timer
            motor_up <= 1'b0;        // Motor off
            motor_down <= 1'b0;      // Motor off
            door_open <= 1'b0;       // Door closed
            timer_count <= 4'b0000;  // Reset timer count
        end else begin
            case (state)
                IDLE: begin
                    if (floor_req != 4'b0000) begin
                        if (comparator_out[3] == 1'b1) begin // If requested floor is above current floor
                            motor_up <= 1'b1;
                            motor_down <= 1'b0;
                            state <= MOVE_UP; // Transition to MOVE_UP state
                        end else if (comparator_out[3] == 1'b0) begin // If requested floor is below current floor
                            motor_up <= 1'b0;
                            motor_down <= 1'b1;
                            state <= MOVE_DOWN; // Transition to MOVE_DOWN state
                        end
                    end
                end
                
                MOVE_UP: begin
                    next_floor <= current_floor + 1; // Move up one floor
                    timer <= timer + 1; // Increment timer
                    if (timer >= 8'd100) begin // Wait for a defined time (e.g., 100 clock cycles)
                        motor_up <= 1'b0; // Stop motor
                        state <= OPEN_DOOR; // Transition to OPEN_DOOR state
                        timer <= 8'b00000000; // Reset timer
                    end
                end
                
                MOVE_DOWN: begin
                    next_floor <= current_floor - 1; // Move down one floor
                    timer <= timer + 1; // Increment timer
                    if (timer >= 8'd100) begin // Wait for a defined time (e.g., 100 clock cycles)
                        motor_down <= 1'b0; // Stop motor
                        state <= OPEN_DOOR; // Transition to OPEN_DOOR state
                        timer <= 8'b00000000; // Reset timer
                    end
                end

                OPEN_DOOR: begin
                    door_open <= 1'b1; // Open the door
                    timer <= timer + 1; // Increment timer
                    if (timer >= 8'd50) begin // Wait for a defined time (e.g., 50 clock cycles)
                        door_open <= 1'b0; // Close the door
                        state <= CLOSE_DOOR; // Transition to CLOSE_DOOR state
                        timer <= 8'b00000000; // Reset timer
                    end
                end
                
                CLOSE_DOOR: begin
                    if (floor_req != 4'b0000) begin
                        state <= IDLE; // Go back to IDLE to handle new requests
                    end
                end
                
                default: state <= IDLE; // Default state
            endcase
        end
    end

    // Timer count output (this can be used for debugging or further processing)
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            timer_count <= 4'b0000; // Reset timer count
        end else begin
            timer_count <= timer[3:0]; // Output lower 4 bits of the timer
        end
    end

endmodule
