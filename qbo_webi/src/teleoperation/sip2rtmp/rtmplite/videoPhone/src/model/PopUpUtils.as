package model
{
	import flash.display.DisplayObject;

	import mx.collections.ArrayCollection;
	import mx.core.Application;
	//import mx.core.FlexGlobals; // NOTE: use this import for Flex 4.x and higher
	import mx.core.IChildList;
	import mx.core.UIComponent;
	import mx.managers.PopUpManager;

	public class PopUpUtils
	{
		/**
		 * Returns all the popups inside an application. Only the popups whose base
		 * class is UIComponent are returned.
		 *
		 * @param applicationInstance
		 *   Application instance. If null, Application.application is used.
		 * @param onlyVisible
		 *   If true, considers only the visible popups.
		 * @return All the popups in the specified application.
		 */
		public static function getAllPopups(applicationInstance: Object = null,
			onlyVisible: Boolean = false): ArrayCollection
		{
			var result: ArrayCollection = new ArrayCollection();

			if (applicationInstance == null)
			{
				// NOTE: use this line for Flex 4.x and higher
				//applicationInstance = FlexGlobals.topLevelApplication;

				// NOTE: use this line for Flex 3.x and lower
				applicationInstance = Application.application;
			}

			var rawChildren: IChildList = applicationInstance.systemManager.rawChildren;

			for (var i: int = 0; i < rawChildren.numChildren; i++)
			{
				var currRawChild: DisplayObject = rawChildren.getChildAt(i);

				if ((currRawChild is UIComponent) && UIComponent(currRawChild).isPopUp)
				{
					if (!onlyVisible || UIComponent(currRawChild).visible)
					{
						result.addItem(currRawChild);
					}
				}
			}

			return result;
		}

		/**
		 * Checks if an application has visible popups. Only the popups whose base
		 * class is UIComponent are considered.
		 *
		 * @param applicationInstance
		 *   Application instance. If null, Application.application is used.
		 * @return True if there are visible popups in the specified application,
		 *         false otherwise.
		 */
		public static function hasVisiblePopups(applicationInstance: Object = null): Boolean
		{
			if (applicationInstance == null)
			{
				// NOTE: use this line for Flex 4.x and higher
				//applicationInstance = FlexGlobals.topLevelApplication;

				// NOTE: use this line for Flex 3.x and lower
				applicationInstance = Application.application;
			}

			var rawChildren: IChildList = applicationInstance.systemManager.rawChildren;

			for (var i: int = 0; i < rawChildren.numChildren; i++)
			{
				var currRawChild: DisplayObject = rawChildren.getChildAt(i);

				if ((currRawChild is UIComponent) && UIComponent(currRawChild).isPopUp
					&& UIComponent(currRawChild).visible)
				{
					return true;
				}
			}

			return false;
		}

		/**
		 * Closes all the popups belonging to an application. Only the popups
		 * whose base class is UIComponent are considered.
		 *
		 * @param applicationInstance
		 *   Application instance. If null, Application.application is used.
		 * @return The list of the closed popups.
		 */
		public static function closeAllPopups(applicationInstance: Object = null): ArrayCollection
		{
			var allPopups: ArrayCollection = getAllPopups(applicationInstance);

			for each (var currPopup: UIComponent in allPopups)
			{
				PopUpManager.removePopUp(currPopup);
			}

			return allPopups;
		}
	}
}
