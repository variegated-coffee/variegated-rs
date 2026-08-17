//! A stack of open menus.

use crate::ListNav;

/// One level of an open menu: which menu it is, and where the selection sits in it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MenuFrame<Id> {
    /// Which menu this level is showing. The caller's own id type.
    pub id: Id,
    /// Where the selection sits in it.
    pub nav: ListNav,
}

/// Where the user is in a menu, as a stack.
///
/// A stack rather than a flat state with a hard-coded "back" destination, because a
/// destination and a return are not the same thing: the Silvia encodes back three
/// incompatible ways -- a per-menu-type constructor, a one-deep boxed parent slot, and a
/// pair of copied fields -- and the constructor discards the parent's selection every time.
///
/// Backed by `[Option<MenuFrame<Id>>; DEPTH]` rather than requiring `Id: Default`, so it
/// works with a bare enum. `Copy` when `Id: Copy`, which the GS3's `Watch` payload needs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MenuStack<Id, const DEPTH: usize> {
    frames: [Option<MenuFrame<Id>>; DEPTH],
    depth: usize,
}

impl<Id: Copy, const DEPTH: usize> Default for MenuStack<Id, DEPTH> {
    fn default() -> Self {
        Self::closed()
    }
}

impl<Id: Copy, const DEPTH: usize> MenuStack<Id, DEPTH> {
    /// No menu open.
    ///
    /// **Depth zero *is* "closed"** -- there is no separate `Option<MenuStack>` wrapper,
    /// because a second way to say the same thing is a second thing two sides can disagree
    /// about.
    pub fn closed() -> Self {
        Self { frames: [None; DEPTH], depth: 0 }
    }

    /// Open `root` at its first row.
    pub fn open(root: Id) -> Self {
        let mut stack = Self::closed();
        stack.push(root);
        stack
    }

    /// Whether any menu is open.
    pub const fn is_open(&self) -> bool {
        self.depth > 0
    }

    /// How many levels are open.
    pub const fn depth(&self) -> usize {
        self.depth
    }

    /// The level being shown.
    pub fn top(&self) -> Option<&MenuFrame<Id>> {
        self.frames.get(self.depth.checked_sub(1)?)?.as_ref()
    }

    /// The level being shown, to move its selection.
    pub fn top_mut(&mut self) -> Option<&mut MenuFrame<Id>> {
        let index = self.depth.checked_sub(1)?;
        self.frames.get_mut(index)?.as_mut()
    }

    /// Open a submenu at its first row, leaving the parent's selection where it is.
    ///
    /// Returns `false` if the stack is full, having changed nothing.
    pub fn push(&mut self, id: Id) -> bool {
        if self.depth >= DEPTH {
            return false;
        }
        self.frames[self.depth] = Some(MenuFrame { id, nav: ListNav::new() });
        self.depth += 1;
        true
    }

    /// Leave the current menu. At the root, that closes the menu entirely.
    ///
    /// The popped frame is cleared rather than left behind, so a closed stack always
    /// compares equal to a freshly closed one -- the GS3 publishes this over a `Watch` and
    /// sends only on change.
    pub fn pop(&mut self) {
        if self.depth == 0 {
            return;
        }
        self.depth -= 1;
        self.frames[self.depth] = None;
    }

    /// Close every level.
    pub fn close(&mut self) {
        while self.is_open() {
            self.pop();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ListGeometry;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum Id { Root, Sub }

    fn geo(total: usize) -> ListGeometry {
        ListGeometry { total_rows: total, visible_rows: 4, wrap: true }
    }

    #[test]
    fn closed_is_closed_and_open_is_open() {
        let stack: MenuStack<Id, 4> = MenuStack::closed();
        assert!(!stack.is_open());
        assert_eq!(stack.depth(), 0);
        assert!(stack.top().is_none());

        let stack: MenuStack<Id, 4> = MenuStack::open(Id::Root);
        assert!(stack.is_open());
        assert_eq!(stack.depth(), 1);
        assert_eq!(stack.top().map(|f| f.id), Some(Id::Root));
    }

    #[test]
    fn push_preserves_the_parents_selection() {
        // The Silvia's `get_back_state` returned `ListMenuState::new()`, so coming back from
        // a submenu always landed on row 0 of the parent.
        let mut stack: MenuStack<Id, 4> = MenuStack::open(Id::Root);
        stack.top_mut().unwrap().nav.down(geo(5));
        stack.top_mut().unwrap().nav.down(geo(5));
        assert_eq!(stack.top().unwrap().nav.selected(), 2);

        assert!(stack.push(Id::Sub));
        assert_eq!(stack.top().unwrap().nav.selected(), 0, "a fresh submenu starts at the top");

        stack.pop();
        assert_eq!(stack.top().map(|f| f.id), Some(Id::Root));
        assert_eq!(stack.top().unwrap().nav.selected(), 2, "the parent kept its row");
    }

    #[test]
    fn popping_the_root_closes_the_menu() {
        let mut stack: MenuStack<Id, 4> = MenuStack::open(Id::Root);
        stack.pop();
        assert!(!stack.is_open());
        stack.pop();
        assert!(!stack.is_open(), "popping a closed stack is a no-op, not an underflow");
    }

    #[test]
    fn a_popped_stack_equals_a_fresh_one() {
        // The GS3 publishes this over a Watch and only sends on change. If a popped frame
        // stayed behind, a closed menu would not compare equal to a closed menu and every
        // close would publish a spurious update.
        let mut stack: MenuStack<Id, 4> = MenuStack::open(Id::Root);
        stack.top_mut().unwrap().nav.down(geo(5));
        stack.pop();
        assert_eq!(stack, MenuStack::closed());
    }

    #[test]
    fn push_past_the_end_is_refused_and_changes_nothing() {
        let mut stack: MenuStack<Id, 2> = MenuStack::open(Id::Root);
        assert!(stack.push(Id::Sub));
        let before = stack;
        assert!(!stack.push(Id::Sub), "a full stack refuses");
        assert_eq!(stack, before, "and is left exactly as it was");
    }

    #[test]
    fn close_empties_any_depth() {
        let mut stack: MenuStack<Id, 4> = MenuStack::open(Id::Root);
        stack.push(Id::Sub);
        stack.push(Id::Sub);
        stack.close();
        assert_eq!(stack, MenuStack::closed());
    }
}
