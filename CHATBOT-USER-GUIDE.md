# RAG Chatbot User Guide

## 🤔 What is Full Textbook vs Selected Text Mode?

### 📚 **Full Textbook Mode** (Default)
**What it does:**  
Searches through the **ENTIRE textbook** (all 13 weeks, all chapters) to answer your question.

**When to use:**
- General questions about any topic
- Comparing concepts from different chapters
- Overview questions
- When you're not sure where the answer is

**Example:**
- ❓ "What are the key components of ROS2?"
- ✅ Searches ALL chapters → Finds relevant sections → Gives comprehensive answer

---

### 📄 **Selected Text Mode**
**What it does:**  
Only searches in the **specific text you highlighted** to answer your question.

**When to use:**
- You're reading a specific section
- Want detailed explanations of that exact content
- Need clarification on particular paragraphs
- Want to focus on one topic without noise from other chapters

**How it works:**
1. **Select/Highlight text** on the page (minimum 20 characters)
2. A **floating panel appears** next to the chat button
3. Click **"Selected Text Only"** mode
4. Ask your question
5. Get answers **only from that selection**

**Example:**
- You're reading the "Sensor Fusion" section
- Highlight 2-3 paragraphs about Kalman filters
- Ask: "How does this filtering technique reduce noise?"
- ✅ Gets answer ONLY from your highlighted text, not from other chapters

---

## 🎨 UI/UX Improvements Made

### Problems Fixed:

#### ❌ **Old Design Issues:**
1. Context selector appeared INSIDE chat window (cluttered)
2. Purple color had poor contrast (hard to read)
3. Selection disappeared when clicking close
4. Minimum 50 characters was too restrictive
5. Generic error messages (just "HTTP 422")

#### ✅ **New Design Solutions:**

1. **Floating Panel**
   - Context selector now appears as a **separate floating panel**
   - Positioned next to chat button (bottom-right)
   - Stays visible even when chat is closed
   - Can be dismissed independently

2. **Better Colors**
   - Replaced purple (#667eea) with **blue/cyan (#0EA5E9)**
   - WCAG AA compliant contrast ratios
   - Clear visual hierarchy
   - Professional appearance

3. **Persistent Selection**
   - Selection stays active until you click ✕
   - Can close chat and reopen - selection persists
   - "Selection Active" indicator in chat header
   - Easy to re-access by reopening floating panel

4. **Relaxed Validation**
   - Minimum reduced from 50 → **20 characters**
   - About 4-5 words minimum
   - More flexible for focused questions

5. **Clear Error Messages**
   - Instead of: "HTTP 422: Unprocessable Content"
   - Now shows: "Your selection is too short. Please select at least 20 characters..."
   - Includes helpful tips

---

## 🔄 How to Use Selected Text Feature

### Step 1: Select Text
```
1. Read any page in the textbook
2. Highlight/select text with your mouse
3. Must be at least 20 characters (4-5 words)
```

### Step 2: Floating Panel Appears
```
📌 A blue panel appears next to the chat button showing:
   - Preview of your selected text
   - Character count
   - Two mode buttons
```

### Step 3: Choose Mode
```
Option A: 📚 Full Textbook
   → Searches entire textbook (all chapters)
   
Option B: 📄 Selected Text Only  
   → Searches ONLY in your highlighted text
```

### Step 4: Ask Question
```
1. Click chat button (💬)
2. Type your question
3. Get answer based on chosen mode
```

### Step 5: Clear Selection (Optional)
```
Click the ✕ button on the floating panel to:
- Remove the selection
- Return to full textbook mode
- Hide the floating panel
```

---

## 💡 Usage Examples

### Example 1: General Question (Full Mode)
```
Scenario: Want to learn about ROS2

Steps:
1. Don't select any text
2. Open chat
3. Ask: "What are the key components of ROS2?"

Result: ✅ Searches all chapters, gives comprehensive answer
```

### Example 2: Focused Question (Selected Mode)
```
Scenario: Confused about a specific paragraph

Steps:
1. Reading "Forward Kinematics" section
2. Highlight 3 paragraphs about transformation matrices
3. Floating panel appears
4. Click "📄 Selected Text Only"
5. Ask: "Can you explain this transformation in simpler terms?"

Result: ✅ Explains ONLY using your highlighted paragraphs
```

### Example 3: Comparing Sections
```
Scenario: Want to understand differences

Steps:
1. Select text about "Forward Kinematics"
2. Click "📄 Selected Text Only"
3. Ask: "How is this different from inverse kinematics?"

Result: ⚠️ Can only answer about forward kinematics
        (inverse kinematics not in selection)

Better approach:
1. Don't select text (use full mode)
2. Ask: "What's the difference between forward and inverse kinematics?"

Result: ✅ Compares both concepts from entire textbook
```

---

## 🐛 Troubleshooting

### Issue: "Selection too short" error
**Cause:** Selected less than 20 characters  
**Fix:** Select more text (at least 4-5 words)

### Issue: Floating panel disappeared
**Cause:** Clicked the ✕ button to clear selection  
**Fix:** Select text again to bring it back

### Issue: Can't see floating panel
**Cause:** No text selected or chat is open  
**Fix:** 
- Close chat window first
- Select text on the page
- Panel will appear next to chat button

### Issue: Getting irrelevant answers
**Cause:** Wrong mode selected
**Fix:**
- For broad questions → Use "📚 Full Textbook"
- For specific text → Use "📄 Selected Text Only"

### Issue: "Cannot connect to server"
**Cause:** Backend not running
**Fix:** 
- Check if backend is deployed
- Verify API URL in settings
- See DEPLOYMENT.md for setup

---

## 🎯 Best Practices

### ✅ DO:
- Use **Full Mode** for general questions
- Use **Selected Mode** for clarifying specific paragraphs
- Select **meaningful chunks** (paragraphs/sections)
- Clear selection when switching topics
- Keep selections focused (not entire pages)

### ❌ DON'T:
- Select single words or very short phrases
- Keep old selections when asking new topics
- Use selected mode for questions outside your selection
- Select entire pages (defeats the purpose)

---

## 📊 Mode Comparison

| Feature | Full Textbook | Selected Text |
|---------|---------------|---------------|
| **Search Scope** | All 13 weeks | Only your selection |
| **Use Case** | General questions | Specific clarifications |
| **Answer Quality** | Comprehensive | Focused & precise |
| **Speed** | Slightly slower | Faster |
| **Best For** | Learning new topics | Understanding specific content |

---

## 🆘 Quick Help

**Q: Which mode should I use?**  
A: Not sure where answer is? → Full Mode. Reading specific section? → Selected Mode.

**Q: How do I re-open the selection panel?**  
A: Close the chat window - the floating panel will reappear if you have text selected.

**Q: Can I change modes mid-conversation?**  
A: Yes! Click the mode buttons in the floating panel anytime.

**Q: Does my selection persist across page navigations?**  
A: No, selecting new text replaces the previous selection. It's meant for per-page usage.

---

## 🚀 Pro Tips

1. **For Deep Understanding:** Select section → Ask multiple follow-up questions in selected mode
2. **For Comparisons:** Use full mode (no selection)
3. **For Definitions:** Select paragraph with term → Ask "Explain this concept"
4. **For Examples:** Select theoretical explanation → Ask "Give me a practical example"
5. **For Simplification:** Select complex text → Ask "Explain this in simpler terms"

---

**Need more help?** Check the [Deployment Guide](DEPLOYMENT.md) or [Technical Documentation](specs/002-rag-chatbot/)
