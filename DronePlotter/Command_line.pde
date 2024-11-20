void keyPressed() {
if (keyCode == ENTER) {
    sp.write(inputText);
    lastInputText = inputText;  // Save the current input text
    inputText = "";             // Clear the input field
    cursorPos = 0;              // Reset cursor position
  }
  
  // Handle Arrow Up key: restore last input text
  if (keyCode == UP) {
    inputText = lastInputText;  // Restore the last saved input text
    cursorPos = inputText.length();  // Set cursor to the end of the text
  }
  
  // Handle Arrow Left key: move cursor left (if not at the beginning)
  if (keyCode == LEFT && cursorPos > 0) {
    cursorPos--;  // Move cursor position to the left
  }
  
  // Handle Arrow Right key: move cursor right (if not at the end)
  if (keyCode == RIGHT && cursorPos < inputText.length()) {
    cursorPos++;  // Move cursor position to the right
  }
  
  // Handle Backspace key: remove the last character
  if (keyCode == BACKSPACE && inputText.length() > 0 && cursorPos > 0) {
    inputText = inputText.substring(0, cursorPos - 1) + inputText.substring(cursorPos);  // Remove character at cursor position
    cursorPos--;  // Move cursor to the left after deleting
  }
  
  // Handle other keys: add typed character to the input text
  else if (keyCode != SHIFT && keyCode != CONTROL && keyCode != ALT && keyCode != RIGHT && keyCode != LEFT) {
    inputText = inputText.substring(0, cursorPos) + key + inputText.substring(cursorPos);  // Insert character at the cursor
    cursorPos++;  // Move cursor position after the inserted character
  }
}
