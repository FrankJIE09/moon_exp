import pygame
import sys
import os

# --- Basic Settings ---
SCREEN_WIDTH = 1000  # Window width
SCREEN_HEIGHT = 800 # Window height
FONT_SIZE = 18      # Slightly smaller font for potentially more columns
PADDING = 15        # Padding around elements
ELEMENT_GAP = 10    # NEW: Gap between text label and its indicator light/bar

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (200, 0, 0)
GREEN = (0, 200, 0)
BLUE = (50, 50, 200) # Adjusted blue for better contrast
GRAY = (150, 150, 150)
AXIS_BAR_COLOR_POS = (0, 180, 0) # Slightly different green for axis bar
AXIS_BAR_COLOR_NEG = (180, 0, 0) # Slightly different red for axis bar

# --- Initialize Pygame ---
pygame.init()
pygame.joystick.init() # Initialize joystick module

# --- Screen Setup ---
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Gamepad Status Display")

# --- Font Setup ---
font = None
try:
    # Try a common system font first
    font = pygame.font.SysFont('Arial', FONT_SIZE)
    print(f"Using font: Arial")
except:
    # Fallback to Pygame's default font
    try:
        font = pygame.font.Font(None, FONT_SIZE + 2) # Default might need slight size adjustment
        print("Using font: Pygame Default")
    except Exception as e:
        print(f"Error loading any font: {e}")
        pygame.quit()
        sys.exit("Font loading failed.")


clock = pygame.time.Clock()

# --- Detect and Initialize Joysticks ---
joysticks = []
num_joysticks_detected = pygame.joystick.get_count()
print(f"Detected {num_joysticks_detected} controller(s).")

if num_joysticks_detected < 1:
    print("Error: No controllers detected. Please connect a controller and restart.")
    pygame.quit()
    sys.exit()

# Initialize all detected joysticks
for i in range(num_joysticks_detected):
    try:
        joystick = pygame.joystick.Joystick(i)
        joystick.init()
        joysticks.append(joystick)
        print(f"Successfully initialized Controller {joystick.get_id()}: {joystick.get_name()}")
        print(f"  Axes: {joystick.get_numaxes()}")
        print(f"  Buttons: {joystick.get_numbuttons()}")
        print(f"  Hats: {joystick.get_numhats()}")
        # print(f"  Balls: {joystick.get_numballs()}") # Balls are uncommon
    except pygame.error as e:
        print(f"Failed to initialize Controller {i}: {e}")

if not joysticks:
    print("Error: Could not initialize any valid controllers.")
    pygame.quit()
    sys.exit()

# --- Calculate Layout Based on Number of Joysticks ---
num_joysticks_active = len(joysticks)
COL_WIDTH = SCREEN_WIDTH // max(1, num_joysticks_active) # Divide screen width evenly

# --- Helper Function: Draw Text ---
def draw_text(surface, text, x, y, color=WHITE, bg_color=None):
    text_surface = font.render(text, True, color, bg_color)
    text_rect = text_surface.get_rect()
    text_rect.topleft = (x, y)
    surface.blit(text_surface, text_rect)
    # MODIFIED: Return the entire rect object
    return text_rect

# --- Helper Function: Draw Axis Status (with visual bar) ---
def draw_axis_status(surface, axis_idx, axis_value, x, y, available_width):
    base_y = y
    # Limit precision for display
    display_value = f"{axis_value:.3f}"
    # MODIFIED: Get the text rect
    text_rect = draw_text(surface, f"Axis {axis_idx}: {display_value}", x, base_y)

    # Visual Bar Configuration
    # MODIFIED: Calculate bar position relative to text
    bar_height = FONT_SIZE * 0.8
    bar_x = text_rect.right + ELEMENT_GAP # Position bar next to text + gap
    bar_y = text_rect.centery - bar_height / 2 # Vertically center bar with text

    # MODIFIED: Calculate bar width based on remaining space in the column
    max_bar_x = x + available_width - PADDING # Right edge limit for the bar
    bar_width = max(30, max_bar_x - bar_x) # Ensure minimum width, fill remaining space

    # Draw background/border
    pygame.draw.rect(surface, GRAY, (bar_x, bar_y, bar_width, bar_height), 1)
    # Calculate fill width (-1 to 1 mapped to 0 to bar_width)
    fill_width = (axis_value + 1) / 2 * bar_width
    # Draw fill bar
    fill_rect = pygame.Rect(bar_x, bar_y, fill_width, bar_height)
    pygame.draw.rect(surface, AXIS_BAR_COLOR_POS if axis_value >= 0 else AXIS_BAR_COLOR_NEG, fill_rect)

    # Draw center marker
    center_x = bar_x + bar_width / 2
    pygame.draw.line(surface, WHITE, (center_x, bar_y), (center_x, bar_y + bar_height), 1)

    # MODIFIED: Return height from the rect
    return text_rect.height # Return vertical space used

# --- Helper Function: Draw Button Status (with visual circle) ---
def draw_button_status(surface, button_idx, button_state, x, y, available_width):
    # MODIFIED: Get the text rect
    text_rect = draw_text(surface, f"Button {button_idx}:", x, y)

    # Visual Circle Configuration
    circle_radius = FONT_SIZE * 0.6
    # MODIFIED: Calculate circle center relative to text
    circle_x = text_rect.right + ELEMENT_GAP + circle_radius # Position circle next to text + gap + radius
    circle_y = text_rect.centery # Vertically center circle with text

    color = GREEN if button_state else RED
    pygame.draw.circle(surface, color, (circle_x, circle_y), circle_radius)
    pygame.draw.circle(surface, WHITE, (circle_x, circle_y), circle_radius, 1) # Border

    # MODIFIED: Return height from the rect
    return text_rect.height

# --- Helper Function: Draw Hat (DPad) Status ---
def draw_hat_status(surface, hat_idx, hat_value, x, y):
    # hat_value is a tuple (x, y): e.g., (0, 0) center, (1, 0) right, (-1, 0) left, (0, 1) up, (0, -1) down
    # MODIFIED: Get the text rect, though not strictly needed here unless adding visuals later
    text_rect = draw_text(surface, f"Hat {hat_idx}: {hat_value}", x, y)
    # Optional: Add a more visual representation like arrows later if desired
    # MODIFIED: Return height from the rect
    return text_rect.height

# --- Main Loop ---
running = True
while running:
    # --- Event Handling ---
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        # Optional: Handle joystick connection/disconnection events (pygame.JOYDEVICEADDED, pygame.JOYDEVICEREMOVED)
        # This requires more complex logic to dynamically update the 'joysticks' list and recalculate layout.

    # --- Clear Screen ---
    screen.fill(BLACK)

    # --- Draw Info for Each Joystick ---
    for i, joystick in enumerate(joysticks):
        start_x = PADDING + i * COL_WIDTH # Starting X for this controller's column
        col_inner_width = COL_WIDTH - 2 * PADDING # Usable width inside the column padding
        current_y = PADDING               # Current Y position for drawing

        # --- Display Controller Name ---
        # Use the modified draw_text, get rect
        controller_name = f"Controller {joystick.get_id()}: {joystick.get_name()}"
        name_rect = draw_text(screen, controller_name, start_x, current_y, BLUE)
        current_y += name_rect.height + PADDING // 2 # Use height from rect
        pygame.draw.line(screen, BLUE, (start_x, current_y), (start_x + col_inner_width, current_y), 1)
        current_y += PADDING // 2

        # --- Display Axes ---
        num_axes = joystick.get_numaxes()
        if num_axes > 0:
            axis_title_rect = draw_text(screen, "Axes:", start_x, current_y, WHITE) # Get rect
            current_y += axis_title_rect.height + 5 # Use height from rect
            for j in range(num_axes):
                axis_value = joystick.get_axis(j)
                # Treat very small values near zero as zero to mitigate stick drift display
                if abs(axis_value) < 0.02:
                    axis_value = 0.0
                # Pass col_inner_width for calculations inside
                axis_h = draw_axis_status(screen, j, axis_value, start_x, current_y, col_inner_width)
                current_y += axis_h + 3 # Spacing between items
            current_y += PADDING // 2 # Spacing after section

        # --- Display Buttons ---
        num_buttons = joystick.get_numbuttons()
        if num_buttons > 0:
            button_title_rect = draw_text(screen, "Buttons:", start_x, current_y, WHITE) # Get rect
            current_y += button_title_rect.height + 5 # Use height from rect
            for j in range(num_buttons):
                button_state = joystick.get_button(j)
                # Pass col_inner_width for calculations inside (though button doesn't use it now)
                button_h = draw_button_status(screen, j, button_state, start_x, current_y, col_inner_width)
                current_y += button_h + 3
            current_y += PADDING // 2

        # --- Display Hats (DPads) ---
        num_hats = joystick.get_numhats()
        if num_hats > 0:
            hat_title_rect = draw_text(screen, "Hats (DPad):", start_x, current_y, WHITE) # Get rect
            current_y += hat_title_rect.height + 5 # Use height from rect
            for j in range(num_hats):
                hat_value = joystick.get_hat(j)
                hat_h = draw_hat_status(screen, j, hat_value, start_x, current_y)
                current_y += hat_h + 3
            current_y += PADDING // 2

        # --- (Optional) Display Balls ---
        # num_balls = joystick.get_numballs()
        # if num_balls > 0:
        #     # ... drawing code for balls ...
        #     pass

    # --- Update Screen Display ---
    pygame.display.flip()

    # --- Cap Frame Rate ---
    clock.tick(60) # Limit to 60 FPS

# --- Clean Up and Exit ---
print("Exiting application...")
pygame.joystick.quit()
pygame.quit()
sys.exit()