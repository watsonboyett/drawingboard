// Copyright 2010 William Malone (www.williammalone.com)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*jslint browser: true */
/*global G_vmlCanvasManager */

var drawingApp = (function () {

	"use strict";

	var colors = new Array();
		colors[0] = "#000000"; 	// black
		colors[1] = "#986928"; 	// brown
		colors[2] = "#B22222"; 	// red
		colors[3] = "#FF4500"; 	// orange
		colors[4] = "#ffcf33"; 	// yellow
		colors[5] = "#659b41"; 	// green
		colors[6] = "#4169E1";	// blue
		colors[7] = "#cb3594";	// violet
		colors[8] = "#808080";	// grey
		colors[9] = "#B8860B"; 	// gold
	
	var canvas,
		context,
		imgUrl,
		
		enableTouch = true,
		
		loadResTotal = 13,
		loadResCurNum = 0,
		crayonImagePR = new Image(),
		markerImagePR = new Image(),
		eraserImagePR = new Image(),
		crayonImagePL = new Image(),
		markerImagePL = new Image(),
		eraserImagePL = new Image(),
		crayonTextureImage = new Image(),
		colorsTitleImage = new Image(),
		toolsTitleImage = new Image(),
		sizeTitleImage = new Image(),
		newTitleImage = new Image(),
		saveTitleImage = new Image(),
		saveDoneTitleImage = new Image(),
		
		clickX = [],
		clickY = [],
		clickColor = [],
		clickTool = [],
		clickSize = [],
		clickDrag = [],
		paint = false,
		saveNeeded = false,
		
		curColor = colors[5],
		curTool = "crayon",
		curSize = "large",
		
		canvasWidth = 800,
		canvasHeight = 600,
		paperStartX = 100,
		paperStartY = 50,
		paperWidth = canvasWidth - paperStartX*2, 	// 800 - 50*2 = 700
		paperHeight = canvasHeight - paperStartY*2, 	// 600 - 50*2 = 500
		
		mediumImageWidth = paperStartX,
		mediumImageHeight = paperStartY,
		mediumOffsetX = mediumImageWidth/3,
		mediumStartX = -mediumOffsetX,
		mediumStartY = (canvasHeight - paperHeight)/2, 	// (600-500)/2 = 50
		
		toolStartX = paperStartX + paperWidth,
		toolStartY = mediumStartY,
		toolWidth = mediumImageWidth,
		toolHeight = mediumImageHeight,
		
		resetBoxStartX = paperStartX + paperWidth/4 - paperStartX/2,
		resetBoxStartY = 0,
		resetBoxWidth = paperStartX,
		resetBoxHeight = paperStartY,
		
		publishBoxStartX = paperStartX + 3*paperWidth/4 - paperStartX/2,
		publishBoxStartY = 0,
		publishBoxWidth = paperStartX,
		publishBoxHeight = paperStartY,
		
		sizeBoxStartX = paperStartX + paperWidth + paperStartX/5,
		sizeBoxStartY = paperStartY + mediumImageHeight*5,
		sizeBoxWidth = 3*paperStartX/5,
		sizeBoxHeight = mediumImageHeight*5,
		
		sizeObjectCount = 5,
		sizeObjectRadius = {
			huge: 27,
			xlarge: 20,
			large: 14,
			normal: 9,
			small: 5
		},
		
		sizeObjectCoords = {
			hugeCX: sizeBoxStartX + sizeBoxWidth/2,
			hugeCY: sizeBoxStartY + 1*sizeBoxHeight/(sizeObjectCount+1),
			
			xlargeCX: sizeBoxStartX + sizeBoxWidth/2,
			xlargeCY: sizeBoxStartY + 2*sizeBoxHeight/(sizeObjectCount+1),
			
			largeCX: sizeBoxStartX + sizeBoxWidth/2,
			largeCY: sizeBoxStartY + 3*sizeBoxHeight/(sizeObjectCount+1),
			
			normalCX: sizeBoxStartX + sizeBoxWidth/2,
			normalCY: sizeBoxStartY + 4*sizeBoxHeight/(sizeObjectCount+1),
			
			smallCX: sizeBoxStartX + sizeBoxWidth/2,
			smallCY: sizeBoxStartY + 5*sizeBoxHeight/(sizeObjectCount+1)
		},
		
		sizeHotspotRadius = sizeBoxHeight/(sizeObjectCount+1) * 0.5,
		sizeHotspots = {
			hugeXL: sizeObjectCoords.hugeCX - sizeHotspotRadius,
			hugeXU: sizeObjectCoords.hugeCX + sizeHotspotRadius,
			hugeYL: sizeObjectCoords.hugeCY - sizeHotspotRadius,
			hugeYU: sizeObjectCoords.hugeCY + sizeHotspotRadius,
			
			xlargeXL: sizeObjectCoords.xlargeCX - sizeHotspotRadius,
			xlargeXU: sizeObjectCoords.xlargeCX + sizeHotspotRadius,
			xlargeYL: sizeObjectCoords.xlargeCY - sizeHotspotRadius,
			xlargeYU: sizeObjectCoords.xlargeCY + sizeHotspotRadius,
			
			largeXL: sizeObjectCoords.largeCX - sizeHotspotRadius,
			largeXU: sizeObjectCoords.largeCX + sizeHotspotRadius,
			largeYL: sizeObjectCoords.largeCY - sizeHotspotRadius,
			largeYU: sizeObjectCoords.largeCY + sizeHotspotRadius,
			
			normalXL: sizeObjectCoords.normalCX - sizeHotspotRadius,
			normalXU: sizeObjectCoords.normalCX + sizeHotspotRadius,
			normalYL: sizeObjectCoords.normalCY - sizeHotspotRadius,
			normalYU: sizeObjectCoords.normalCY + sizeHotspotRadius,

			smallXL: sizeObjectCoords.smallCX - sizeHotspotRadius,
			smallXU: sizeObjectCoords.smallCX + sizeHotspotRadius,
			smallYL: sizeObjectCoords.smallCY - sizeHotspotRadius,
			smallYU: sizeObjectCoords.smallCY + sizeHotspotRadius
		},
		
		
		// Clears the canvas.
		clearCanvas = function () {
			context.clearRect(0, 0, canvasWidth, canvasHeight);
		},

		newPaper = function () {
			saveDrawing(false);
			
			clickX = [];
			clickY = [];
			clickTool = [];
			clickColor = [];
			clickSize = [];
			clickDrag = [];
		},
		
		savePaper = function () {
		    saveDrawing(true);
		    saveNeeded = false;
		},
		
		saveDrawing = function (doPublish) {
            var image = Canvas2Image.saveAsPNG(canvas, true);
			
			//dunno why this is needed, but it is...
			var ci = document.getElementById("canvasimage")
			ci.parentNode.replaceChild(image,ci);
			image.id = "canvasimage";
			image.hidden = "true";

			
			var loc = '';
			if (doPublish) {
			    loc = '../../gallery/saves/';
			} else {
			    loc = '../../gallery/trash/';
			}
			
			var time = Math.floor(new Date().getTime()/1000);
			var url = './php/saveimage.php',
			    name = time.toString(),
			    img = $('#canvasimage').attr('src');

			$.ajax({
			    type: "POST",
			    url: url,
			    dataType: 'text',
			    data: {
			        name : name + '.png',
			        loc : loc,
			        img : img
			    }
		    });   
			 
		},
		
		drawPaper = function () {
			var lw = context.lineWidth;
			
			// draw paper bounds
			context.lineWidth = 2;
			context.beginPath();
			context.rect(paperStartX, paperStartY, paperWidth, paperHeight);
			context.stroke();
			context.closePath();
			
			// draw canvas bounds
			context.lineWidth = 1;
			context.beginPath();
			context.rect(0, 0, canvasWidth, canvasHeight);
			context.stroke();
			context.closePath();
			
			context.drawImage(colorsTitleImage, paperStartX/8, paperStartY/8, 6*toolWidth/8, 7*toolHeight/8);
			context.drawImage(toolsTitleImage, paperStartX+paperWidth+paperStartX/8, paperStartY/8, 6*toolWidth/8, 7*toolHeight/8);
			context.drawImage(sizeTitleImage, paperStartX+paperWidth+paperStartX/8, sizeBoxStartY-6*toolHeight/8, 6*toolWidth/8, 7*toolHeight/8);

			context.lineWidth = lw;
		},
		
		drawTools = function () {
			context.drawImage(crayonImagePL, toolStartX, toolStartY, toolWidth, toolHeight);
			context.drawImage(markerImagePL, toolStartX, toolStartY+toolHeight, toolWidth, toolHeight);
			context.drawImage(eraserImagePL, toolStartX, toolStartY+toolHeight*2, toolWidth, toolHeight);
		},
		
		drawButtons = function () {			
			// draw clear button bounding box
			context.lineWidth = 1;
			context.beginPath();
			context.rect(resetBoxStartX, resetBoxStartY, resetBoxWidth, resetBoxHeight);
			context.stroke();
			context.closePath();
			context.drawImage(newTitleImage, resetBoxStartX, resetBoxStartY, resetBoxWidth, resetBoxHeight);
			
			// draw publish button bounding box
			context.lineWidth = 1;
			context.beginPath();
			context.rect(publishBoxStartX, publishBoxStartY, publishBoxWidth, publishBoxHeight);
			context.stroke();
			context.closePath();
			if (saveNeeded) {
			    context.drawImage(saveTitleImage, publishBoxStartX, publishBoxStartY, publishBoxWidth, publishBoxHeight);
			} else {
			    context.drawImage(saveDoneTitleImage, publishBoxStartX, publishBoxStartY, publishBoxWidth, publishBoxHeight);
			}
		},
		
		drawSizes = function () {
			var cx = toolStartX+paperStartX/2,
				cr = sizeObjectRadius.huge/2,
				cy = sizeBoxStartY+cr+10;
			var lw = context.lineWidth;
			
			// Draw sizes and box
			context.rect(sizeBoxStartX, sizeBoxStartY, sizeBoxWidth, sizeBoxHeight);
			context.stroke();
			
			context.lineWidth = 2;
			context.beginPath();
			context.arc(sizeObjectCoords.hugeCX, sizeObjectCoords.hugeCY, sizeObjectRadius.huge/2, 0, 2*Math.PI);
			context.closePath();
			context.stroke();
			
			context.beginPath();
			context.arc(sizeObjectCoords.xlargeCX, sizeObjectCoords.xlargeCY, sizeObjectRadius.xlarge/2, 0, 2*Math.PI);
			context.closePath();
			context.stroke();
			
			context.beginPath();
			context.arc(sizeObjectCoords.largeCX, sizeObjectCoords.largeCY, sizeObjectRadius.large/2, 0, 2*Math.PI);
			context.closePath();
			context.stroke();
			
			context.beginPath();
			context.arc(sizeObjectCoords.normalCX, sizeObjectCoords.normalCY, sizeObjectRadius.normal/2, 0, 2*Math.PI);
			context.closePath();
			context.stroke();
			
			context.beginPath();
			context.arc(sizeObjectCoords.smallCX, sizeObjectCoords.smallCY, sizeObjectRadius.small/2, 0, 2*Math.PI);
			context.closePath();
			context.stroke();
		
			context.lineWidth = lw;
		},
		
		// Redraws the canvas.
		redraw = function () {			
			var locX,
				locY,
				radius,
				i,
				selected,

				drawColors = function (toolFun) {
					var colorStartX = mediumStartX + mediumOffsetX,
						colorStartY = mediumStartY,
						colorIncrY = mediumImageHeight;
					
					locY = colorStartY;
					for (var i=0; i<colors.length; i++) {
						selected = (curColor === colors[i]);
						locX = selected ? colorStartX : mediumStartX;
						toolFun(locX, locY, colors[i], selected);
						locY += colorIncrY;
					}
				},
				
				drawCrayon = function (x, y, color, selected) {
					var sx = 56,
						sy = 13,
						sw = 12,
						sw2 = 19,
						sh = 24,
						sh2 = 8;
					context.beginPath();
					context.moveTo(x + sx, y + sy);
					context.lineTo(x + sx, y + sy+sh);
					context.lineTo(x + sx+sw, y + sy+sh);
					context.lineTo(x + sx+sw, y + sy+sh-1);
					context.lineTo(x + sx+sw+sw2, y + sy+sh-sh2);
					context.lineTo(x + sx+sw+sw2, y + sy+sh-sh2*2);
					context.lineTo(x + sx+sw, y + sy+sh-sh2*3-1)
					context.lineTo(x + sx+sw, y + sy);
					context.lineTo(x + sx, y + sy);
					context.closePath();
					context.fillStyle = color;
					context.fill();
					context.stroke();
					
					if (selected) {
						context.drawImage(crayonImagePR, x, y, mediumImageWidth, mediumImageHeight);
					} else {
						context.drawImage(crayonImagePR, 0, 0, 93, 46, x, y, mediumImageWidth, mediumImageHeight);
					}
				},

				drawMarker = function (x, y, color, selected) {
					var sx = 89,
						sy = 26,
						sw = 12,
						sh = 8;
					context.beginPath();
					context.moveTo(x + sx, y + sy);
					//context.lineTo(x + sx, y + sy);
					context.lineTo(x + sx-sw, y + sy-sh);
					context.lineTo(x + sx-sw, y + sy+sh);
					context.closePath();
					context.fillStyle = color;
					context.fill();

					if (selected) {
						context.drawImage(markerImagePR, x, y, mediumImageWidth, mediumImageHeight);
					} else {
						context.drawImage(markerImagePR, 0, 0, 93, 46, x, y, mediumImageWidth	, mediumImageHeight);
					}
				},
				
				drawEraser = function (x, y) {
					context.drawImage(eraserImagePR, x, y, mediumImageWidth, mediumImageHeight);
				};

			// Make sure required resources are loaded before redrawing
			if (loadResCurNum < loadResTotal) {
				return;
			}

			clearCanvas();
			drawPaper();
			
			drawTools();
			drawSizes();
			drawButtons();
			
			if (curTool === "crayon") {
				drawColors(drawCrayon);
			} else if (curTool === "marker") {
				drawColors(drawMarker);
			} else if (curTool === "eraser") {
				drawEraser(mediumStartX + mediumOffsetX, mediumStartY);
			}

			// Draw color on ruler to indicate size
			var locCX,
				locCY,
				locCR;
			switch (curSize) {
				case "small":
					locCX = sizeObjectCoords.smallCX;
					locCY = sizeObjectCoords.smallCY;
					locCR = sizeObjectRadius.small/2;
					break;
				case "normal":
					locCX = sizeObjectCoords.normalCX;
					locCY = sizeObjectCoords.normalCY;
					locCR = sizeObjectRadius.normal/2;
					break;
				case "large":
					locCX = sizeObjectCoords.largeCX;
					locCY = sizeObjectCoords.largeCY;
					locCR = sizeObjectRadius.large/2;
					break;
				case "xlarge":
					locCX = sizeObjectCoords.xlargeCX;
					locCY = sizeObjectCoords.xlargeCY;
					locCR = sizeObjectRadius.xlarge/2;
					break;
				case "huge":
					locCX = sizeObjectCoords.hugeCX;
					locCY = sizeObjectCoords.hugeCY;
					locCR = sizeObjectRadius.huge/2;
					break;
				default:
					break;
			}
			context.beginPath();
			context.arc(locCX, locCY, locCR, 0, 2*Math.PI);
			context.closePath();
			context.fillStyle = curColor;
			context.fill();

			// Keep the drawing in the drawing area
			context.save();
			context.beginPath();
			context.rect(paperStartX, paperStartY, paperWidth, paperHeight);
			context.clip();

			// For each point drawn
			for (i = 0; i < clickX.length; i += 1) {

				// Set the drawing radius
				switch (clickSize[i]) {
				case "small":
					radius = sizeObjectRadius.small;
					break;
				case "normal":
					radius = sizeObjectRadius.normal;
					break;
				case "large":
					radius = sizeObjectRadius.large;
					break;
				case "huge":
					radius = sizeObjectRadius.huge;
					break;
				default:
					break;
				}

				// Set the drawing path
				context.beginPath();
				// If dragging then draw a line between the two points
				if (clickDrag[i] && i) {
					context.moveTo(clickX[i - 1], clickY[i - 1]);
				} else {
					// The x position is moved over one pixel so a circle even if not dragging
					context.moveTo(clickX[i] - 1, clickY[i]);
				}
				context.lineTo(clickX[i], clickY[i]);
				
				// Set the drawing color
				if (clickTool[i] === "eraser") {
					//context.globalCompositeOperation = "destination-out"; // To erase instead of draw over with white
					context.strokeStyle = 'white';
				} else {
					//context.globalCompositeOperation = "source-over";	// To erase instead of draw over with white
					context.strokeStyle = clickColor[i];
				}
				context.lineCap = "round";
				context.lineJoin = "round";
				context.lineWidth = radius;
				context.stroke();
			}
			context.closePath();
			//context.globalCompositeOperation = "source-over";// To erase instead of draw over with white
			context.restore();

			// Overlay a crayon texture (if the current tool is crayon)
			if (curTool === "crayon") {
				context.globalAlpha = 0.3; // No IE support
				context.drawImage(crayonTextureImage, paperStartX, paperStartY, paperWidth, paperHeight);
			}
			context.globalAlpha = 1; // No IE support

		},

		// Adds a point to the drawing array.
		// @param x
		// @param y
		// @param dragging
		addClick = function (x, y, dragging) {
		    saveNeeded = true;
			clickX.push(x);
			clickY.push(y);
			clickTool.push(curTool);
			clickColor.push(curColor);
			clickSize.push(curSize);
			clickDrag.push(dragging);
		},

        mousepress = function (e) {
			var pos = {
			    x: e.pageX - this.offsetLeft,
			    y: e.pageY - this.offsetTop
			};
			press(pos);
        },
        
        touchpress = function (e) {
            var pos = {
                x: e.touches[0].pageX - this.offsetLeft,
				y: e.touches[0].pageY - this.offsetTop
		    };
		    press(pos);
        },
        
		press = function (pos) {
			// Mouse down location
			var sizeHotspotStartX,
			    mouseX = pos.x,
			    mouseY = pos.y;

			if (mouseX < paperStartX) { // Left of the drawing area
				if (mouseX > mediumStartX) {
					var colorInd = Math.floor( (mouseY - mediumStartY) / mediumImageHeight );
					curColor = colors[colorInd];
				}
			} else if (mouseX > paperStartX + paperWidth) { // Right of the drawing area
				
				if (mouseY > toolStartY && mouseY < sizeBoxStartY) {	// tool area
					if (mouseY < toolStartY + toolHeight) {
						curTool = "crayon";
					} else if (mouseY < toolStartY + toolHeight * 2) {
						curTool = "marker";
					} else if (mouseY < toolStartY + toolHeight * 3) {
						curTool = "eraser";
					}
				}
				
				if (mouseY > sizeBoxStartY && mouseY < sizeBoxStartY+sizeBoxHeight) { 	// size area
					if (mouseX > sizeBoxStartX && mouseX < sizeBoxStartX+sizeBoxWidth) {
						if (mouseY > sizeHotspots.hugeYL && mouseY < sizeHotspots.hugeYU) {
							curSize = "huge";
						} else if (mouseY > sizeHotspots.xlargeYL && mouseY < sizeHotspots.xlargeYU) {
							curSize = "xlarge"; 
						} else if (mouseY > sizeHotspots.largeYL && mouseY < sizeHotspots.largeYU) {
							curSize = "large";
						} else if (mouseY > sizeHotspots.normalYL && mouseY < sizeHotspots.normalYU) {
							curSize = "normal";
						} else if (mouseY > sizeHotspots.smallYL && mouseY < sizeHotspots.smallYU) {
							curSize = "small";
						}
					}
				}
				
			} else {	
			
				if (mouseY < paperStartY) {		// above drawing area
					if (mouseX > resetBoxStartX && mouseX < resetBoxStartX + resetBoxWidth) {
						newPaper();
						return;
					} else if (mouseX > publishBoxStartX && mouseX < publishBoxStartX + publishBoxWidth) {
					    savePaper();
					    return;
					}
				
				}
				
			    
			}
			
			paint = true;
			addClick(mouseX, mouseY, false);
			redraw();
		},

        mousedrag = function(e) {
            var pos = {
                x: e.pageX - this.offsetLeft,
				y: e.pageY - this.offsetTop
			};
			drag(pos);
        },
        
        touchdrag = function (e) {
            var pos = {
                x: e.touches[0].pageX - this.offsetLeft,
			    y: e.touches[0].pageY - this.offsetTop
			};
			
			drag(pos);
			// Prevent the whole page from dragging if on mobile
			e.preventDefault();
        },
        
		drag = function (pos) {
			if (paint) {
				var mouseX = pos.x,
				    mouseY = pos.y;
				addClick(mouseX, mouseY, true);
				redraw();
			}
		},

		release = function () {
			paint = false;
			redraw();
		},

		cancel = function () {
			paint = false;
		},
				
		// Add mouse and touch event listeners to the canvas
		createUserEvents = function () {
			// Add mouse event listeners to canvas element
			canvas.addEventListener("mousedown", mousepress, false);
			canvas.addEventListener("mousemove", mousedrag, false);
			canvas.addEventListener("mouseup", release);
			canvas.addEventListener("mouseout", cancel, false);

			// Add touch event listeners to canvas element
			canvas.addEventListener("touchstart", touchpress, false);
			canvas.addEventListener("touchmove", touchdrag, false);
			canvas.addEventListener("touchend", release, false);
			canvas.addEventListener("touchcancel", cancel, false);
		},

		// Calls the redraw function after all neccessary resources are loaded.
		resourceLoaded = function () {
			loadResCurNum += 1;
			if (loadResCurNum === loadResTotal) {
				redraw();
				createUserEvents();
			}
		},

		// Creates a canvas element, loads images, adds events, and draws the canvas for the first time.
		init = function () {
			// Create the canvas (Neccessary for IE because it doesn't know what a canvas element is)
			canvas = document.createElement('canvas');
			canvas.setAttribute('width', canvasWidth);
			canvas.setAttribute('height', canvasHeight);
			//canvas.setAttribute('background', canvasHeight);
			canvas.setAttribute('id', 'canvas');
			document.getElementById('canvasDiv').appendChild(canvas);
			if (typeof G_vmlCanvasManager !== "undefined") {
				canvas = G_vmlCanvasManager.initElement(canvas);
			}
			context = canvas.getContext("2d"); // Grab the 2d canvas context
			// Note: The above code is a workaround for IE 8 and lower. Otherwise we could have used:
			//     context = document.getElementById('canvas').getContext("2d");

			// Load images
			crayonImagePL.onload = resourceLoaded;
			crayonImagePL.src = "images/crayon-outline-pl.png";
			crayonImagePR.onload = resourceLoaded;
			crayonImagePR.src = "images/crayon-outline-pr.png";
			
			markerImagePL.onload = resourceLoaded;
			markerImagePL.src = "images/marker-outline-pl.png";
			markerImagePR.onload = resourceLoaded;
			markerImagePR.src = "images/marker-outline-pr.png";
			
			eraserImagePL.onload = resourceLoaded;
			eraserImagePL.src = "images/eraser-outline-pl.png";
			eraserImagePR.onload = resourceLoaded;
			eraserImagePR.src = "images/eraser-outline-pr.png";

			crayonTextureImage.onload = resourceLoaded;
			crayonTextureImage.src = "images/crayon-texture.png";
			
			colorsTitleImage.onload = resourceLoaded;
			colorsTitleImage.src = "images/colors-title.png";
			toolsTitleImage.onload = resourceLoaded;
			toolsTitleImage.src = "images/tools-title.png";
			sizeTitleImage.onload = resourceLoaded;
			sizeTitleImage.src = "images/size-title.png";
			
			newTitleImage.onload = resourceLoaded;
			newTitleImage.src = "images/new-title.png";
			saveTitleImage.onload = resourceLoaded;
			saveTitleImage.src = "images/save-title.png";
			saveDoneTitleImage.onload = resourceLoaded;
			saveDoneTitleImage.src = "images/save_done-title.png";
		};

	return {
		init: init
	};
}());
